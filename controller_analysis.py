#!/usr/bin/env python3
"""
Pitchlink, LLC 
Copyright (c) 2025 MIT License
This script analyzes ArduPilot log files to extract and recommend tuning parameters for rate and attitude controllers.
Maintained by Pitchlink, LLC.
Mainters: software@pitchlink.org
This script is provided without warranty; use at your own risk. Use common sense and caution when tuning your vehicle. If you need help reach out to us at information@pitchlink.org.
www.pitchl.ink
"""

import sys
import argparse
import logging
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pymavlink import mavutil
from tqdm import tqdm

# --- Configuration Defaults ---
DEFAULT_TARGET_RATE_RMS = 5.0        # deg/s
DEFAULT_TARGET_ATT_ERR  = 2.0        # deg
DEFAULT_MAX_BUMP        = 1.20       # +20%
DEFAULT_TRIM_START      = 5.0        # seconds
DEFAULT_TRIM_END        = 5.0        # seconds
PSDFREQ_MIN = 10     # Hz
PSDFREQ_MAX = 200    # Hz

# --- Setup Logger ---
logger = logging.getLogger('tuner')

# --- Helper: verify file exists ---
def verify_logfile(path):
    if not os.path.isfile(path):
        logger.error('Log file not found: %s', path)
        sys.exit(1)

# --- Delay Estimation ---
def estimate_delay(setpoint, response, fs):
    """
    Estimate actuator delay tau (seconds) via cross-correlation.
    """
    sp = setpoint - np.mean(setpoint)
    rp = response - np.mean(response)
    corr = np.correlate(rp, sp, mode='full')
    lag = corr.argmax() - (len(sp) - 1)
    return lag / fs

# --- Parameter Extraction ---
def extract_old_params(log_path):
    """
    Extract rate PID gains, attitude P gains,
    old gyro filter cutoff, and old feed-forward gains.
    Show progress with tqdm.
    """
    mlog = mavutil.mavlink_connection(log_path, dialect='ardupilotmega', robust_parsing=True)
    rate_gains = {ax: {} for ax in ('roll','pitch','yaw')}
    att_gains  = {ax: {} for ax in ('roll','pitch','yaw')}
    filter_old = {}
    ff_old     = {}

    # Determine schema
    first = mlog.recv_match(type='PARM', blocking=True)
    if not first:
        logger.error('No PARM messages found in log')
        sys.exit(1)
    fields = first.get_fieldnames()
    id_field = next((f for f in fields if f.lower().replace('_','') in ('paramid','name')), None)
    val_field= next((f for f in fields if f.lower().replace('_','') in ('paramvalue','value')), None)
    if not id_field or not val_field:
        logger.error('Cannot identify PARM fields among %s', fields)
        sys.exit(1)

    pcm_rate = {
        'ATC_RAT_RLL_P':('roll','P'), 'ATC_RAT_RLL_I':('roll','I'), 'ATC_RAT_RLL_D':('roll','D'),
        'ATC_RAT_PIT_P':('pitch','P'),'ATC_RAT_PIT_I':('pitch','I'),'ATC_RAT_PIT_D':('pitch','D'),
        'ATC_RAT_YAW_P':('yaw','P'),  'ATC_RAT_YAW_I':('yaw','I'),  'ATC_RAT_YAW_D':('yaw','D')
    }
    pcm_att = {
        'ATC_ANG_RLL_P':('roll','P'), 'ATC_ANG_PIT_P':('pitch','P'), 'ATC_ANG_YAW_P':('yaw','P')
    }
    pcm_ff = {
        'ATC_RAT_RLL_FF':'roll', 'ATC_RAT_PIT_FF':'pitch', 'ATC_RAT_YAW_FF':'yaw'
    }

    # parse all PARM messages
    for msg in tqdm(iter(lambda: mlog.recv_match(type='PARM', blocking=False), None),
                    desc='Extracting PARM', unit='msg'):
        name = getattr(msg, id_field, None)
        val  = getattr(msg, val_field, None)
        if name in pcm_rate:
            ax,k = pcm_rate[name]; rate_gains[ax][k] = val
        elif name in pcm_att:
            ax,k = pcm_att[name]; att_gains[ax][k] = val
        elif name == 'INS_GYRO_FILTER':
            filter_old['gyro_filter_hz'] = val
        elif name in pcm_ff:
            ff_old[pcm_ff[name]] = val

    return rate_gains, att_gains, filter_old, ff_old

# --- Parsing Helpers ---
def trim_df(df, start, end):
    tmax = df['time_s'].max()
    return df[(df['time_s']>=start)&(df['time_s']<=tmax-end)].reset_index(drop=True)

# --- Rate Analysis ---
def sniff_rate_fields(msg):
    names = msg.get_fieldnames(); des,act,out = {},{},{}
    for ax in ('roll','pitch','yaw'):
        letter = ax[0]
        des[ax] = next(f for f in names if f.lower().startswith(letter+'des'))
        act[ax] = next(f for f in names if f.lower()==letter)
        out[ax] = next(f for f in names if f.lower()==letter+'out')
    return des,act,out


def parse_rate(log_path, trim_start, trim_end, apply_delay=False):
    m = mavutil.mavlink_connection(log_path, dialect='ardupilotmega', robust_parsing=True)
    first = m.recv_match(type='RATE', blocking=True)
    des,act,out = sniff_rate_fields(first)
    data = {'time_s':[]}
    for ax in ('roll','pitch','yaw'):
        data[f'{ax}_sp']=[]; data[f'{ax}_act']=[]; data[f'{ax}_out']=[]
    t0=first._timestamp

    # first sample
    data['time_s'].append(0.0)
    for ax in ('roll','pitch','yaw'):
        data[f'{ax}_sp'].append(getattr(first,des[ax])*180/np.pi)
        data[f'{ax}_act'].append(getattr(first,act[ax])*180/np.pi)
        data[f'{ax}_out'].append(getattr(first,out[ax]))

    # rest
    for msg in tqdm(iter(lambda: m.recv_match(type='RATE', blocking=False), None),
                    desc='Parsing RATE', unit='msg'):
        t = msg._timestamp - t0; data['time_s'].append(t)
        for ax in ('roll','pitch','yaw'):
            data[f'{ax}_sp'].append(getattr(msg,des[ax])*180/np.pi)
            data[f'{ax}_act'].append(getattr(msg,act[ax])*180/np.pi)
            data[f'{ax}_out'].append(getattr(msg,out[ax]))

    df = pd.DataFrame(data)
    if apply_delay:
        fs=1/np.mean(np.diff(df['time_s']))
        taus={ax: estimate_delay(df[f'{ax}_sp'], df[f'{ax}_act'], fs) for ax in ('roll','pitch','yaw')}
        tau_med=np.median(list(taus.values()))
        df['time_s']=df['time_s']-tau_med
    for ax in ('roll','pitch','yaw'):
        df[f'{ax}_err']=df[f'{ax}_sp']-df[f'{ax}_act']
    return trim_df(df, trim_start, trim_end)

# --- Attitude Analysis ---
def sniff_att_fields(msg):
    names=msg.get_fieldnames(); des,act={},{}
    for ax in ('roll','pitch','yaw'):
        f_des=f'Des{ax.capitalize()}'
        des[ax]=next(f for f in names if f.lower()==f_des.lower())
        act[ax]=next(f for f in names if f.lower()==ax)
    return des,act


def parse_attitude(log_path, trim_start, trim_end, apply_delay=False):
    m = mavutil.mavlink_connection(log_path, dialect='ardupilotmega', robust_parsing=True)
    first=m.recv_match(type='ATT', blocking=True)
    des,act=sniff_att_fields(first)
    data={'time_s':[]}
    for ax in ('roll','pitch','yaw'):
        data[f'{ax}_sp']=[]; data[f'{ax}_act']=[]
    t0=first._timestamp

    data['time_s'].append(0.0)
    for ax in ('roll','pitch','yaw'):
        data[f'{ax}_sp'].append(getattr(first,des[ax])*180/np.pi)
        data[f'{ax}_act'].append(getattr(first,act[ax])*180/np.pi)

    for msg in tqdm(iter(lambda: m.recv_match(type='ATT', blocking=False), None),
                    desc='Parsing ATT', unit='msg'):
        t=msg._timestamp-t0; data['time_s'].append(t)
        for ax in ('roll','pitch','yaw'):
            data[f'{ax}_sp'].append(getattr(msg,des[ax])*180/np.pi)
            data[f'{ax}_act'].append(getattr(msg,act[ax])*180/np.pi)

    df=pd.DataFrame(data)
    if apply_delay:
        fs=1/np.mean(np.diff(df['time_s']))
        taus={ax: estimate_delay(df[f'{ax}_sp'], df[f'{ax}_act'], fs) for ax in ('roll','pitch','yaw')}
        tau_med=np.median(list(taus.values()))
        df['time_s']=df['time_s']-tau_med
    for ax in ('roll','pitch','yaw'):
        df[f'{ax}_err']=df[f'{ax}_sp']-df[f'{ax}_act']
    return trim_df(df, trim_start, trim_end)

# --- IMU & Filter Recommendation ---
def parse_imu(log_path):
    m = mavutil.mavlink_connection(log_path, dialect='ardupilotmega', robust_parsing=True)
    first=m.recv_match(type='IMU', blocking=True)
    if not first: return pd.DataFrame()
    fields=first.get_fieldnames(); gyro_map={}
    for f in fields:
        lf=f.lower()
        if 'gyrx' in lf: gyro_map['gyro_x']=f
        elif 'gyry' in lf: gyro_map['gyro_y']=f
        elif 'gyrz' in lf: gyro_map['gyro_z']=f
    data={'time_s':[],'gyro_x':[],'gyro_y':[],'gyro_z':[]}
    t0=first._timestamp; data['time_s'].append(0.0)
    for k in gyro_map: data[k].append(getattr(first,gyro_map[k],0.0))
    for msg in tqdm(iter(lambda: m.recv_match(type='IMU', blocking=False), None),
                    desc='Parsing IMU', unit='msg'):
        t=msg._timestamp-t0; data['time_s'].append(t)
        for k in gyro_map:
            data[k].append(getattr(msg,gyro_map[k],0.0))
    return pd.DataFrame(data)


def recommend_filter(imu_df):
    if imu_df.empty: return {}
    fs=1/np.mean(np.diff(imu_df['time_s'])); rec={}
    for ax in ('gyro_x','gyro_y','gyro_z'):
        sig=imu_df[ax]-imu_df[ax].mean(); freqs=np.fft.rfftfreq(len(sig),1/fs); psd=np.abs(np.fft.rfft(sig))
        mask=(freqs>=PSDFREQ_MIN)&(freqs<=PSDFREQ_MAX)
        rec[ax]=(freqs[mask][np.argmax(psd[mask])]*1.2) if mask.any() else 0.0
    return rec

# --- Summaries & Recommendations ---
def summarize(df):
    stats=[]
    for ax in ('roll_err','pitch_err','yaw_err'):
        e=df[ax]; stats.append({'axis':ax.replace('_err',''),'mean':e.mean(),'rms':np.sqrt((e**2).mean()),'max':e.abs().max()})
    return pd.DataFrame(stats)


def recommend_rate(stats, old, target_rms, max_bump):
    rec={}
    for _,r in stats.iterrows():
        ax=r['axis']; scale=r['rms']/target_rms; P_old=old[ax]['P']; I_old=old[ax]['I']; D_old=old[ax]['D']
        P_new=min(P_old*scale,P_old*max_bump)
        rec[ax]={'P_old':P_old,'I_old':I_old,'D_old':D_old,'P_new':P_new,'I_new':P_new*0.1,'D_new':P_new*0.01,'scale':scale}
    return pd.DataFrame.from_dict(rec,orient='index')


def recommend_att(stats, old, target_err, max_bump):
    rec={}
    for _,r in stats.iterrows():
        ax=r['axis']; scale=r['max']/target_err; P_old=old[ax]['P']; P_new=min(P_old*scale,P_old*max_bump)
        rec[ax]={'P_old':P_old,'P_new':P_new,'scale':scale}
    return pd.DataFrame.from_dict(rec,orient='index')


def recommend_ff(df_rate, ff_old):
    rec={}
    for ax in ('roll','pitch','yaw'):
        sp=df_rate[f'{ax}_sp'].values; out=df_rate[f'{ax}_out'].values
        mask=sp!=0
        slope=np.dot(out[mask],sp[mask]) / np.dot(sp[mask],sp[mask]) if mask.sum()>0 else 0.0
        rec[ax]={'FF_old':ff_old.get(ax,'N/A'),'FF_est':slope}
    return pd.DataFrame.from_dict(rec,orient='index')

# --- Main ---
if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Controller log tuner w/ delay & FF estimation')
    parser.add_argument('logfile', help='ArduPilot .BIN log')
    parser.add_argument('-v','--verbose', action='store_true', default=False, help='Verbose')
    parser.add_argument('-f','--filter', action='store_true', default=True,  help='Filter')
    parser.add_argument('-d','--delay',  action='store_true', default=True,  help='Delay')
    parser.add_argument('-o','--output',              help='Output report',    default=None)
    parser.add_argument('--rate-rms',   type=float,    default=DEFAULT_TARGET_RATE_RMS)
    parser.add_argument('--att-err',    type=float,    default=DEFAULT_TARGET_ATT_ERR)
    parser.add_argument('--bump',       type=float,    default=DEFAULT_MAX_BUMP)
    parser.add_argument('--trim-start', type=float,    default=DEFAULT_TRIM_START)
    parser.add_argument('--trim-end',   type=float,    default=DEFAULT_TRIM_END)
    args = parser.parse_args()

    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format='%(asctime)s %(levelname)s %(message)s')

    verify_logfile(args.logfile)
    logger.info('Starting analysis on %s', args.logfile)

    rate_old, att_old, filter_old, ff_old = extract_old_params(args.logfile)
    df_rate = parse_rate(args.logfile, args.trim_start, args.trim_end, apply_delay=args.delay)
    rate_stats = summarize(df_rate)
    rate_rec   = recommend_rate(rate_stats, rate_old, args.rate_rms, args.bump)

    df_att = parse_attitude(args.logfile, args.trim_start, args.trim_end, apply_delay=args.delay)
    att_stats = summarize(df_att)
    att_rec   = recommend_att(att_stats, att_old, args.att_err, args.bump)

    report = [
        '=== Old Gyro Filter ===', f"gyro_filter_hz: {filter_old.get('gyro_filter_hz','N/A')}"
    ]
    report += [
        '=== Old FF Gains ===', str(ff_old),
        '=== Rate Error Summary ===',      rate_stats.to_string(index=False),
        '=== Rate Gain Rec ===',           rate_rec.to_string(),
        '=== Attitude Error Summary ===',   att_stats.to_string(index=False),
        '=== Attitude Gain Rec ===',        att_rec.to_string()
    ]
    ff_rec = recommend_ff(df_rate, ff_old)
    report += ['=== FF Estimation ===', ff_rec.to_string()]

    if args.filter:
        imu_df   = parse_imu(args.logfile)
        filt_rec = recommend_filter(imu_df)
        report  += ['=== Filter Rec (Hz) ===', pd.Series(filt_rec).to_string()]

    if args.delay:
        fs      = 1/np.mean(np.diff(df_rate['time_s']))
        delays  = {ax: estimate_delay(df_rate[f'{ax}_sp'], df_rate[f'{ax}_act'], fs) for ax in ('roll','pitch','yaw')}
        report += ['=== Delay Estimates (s) ===', str(delays)]

    full_report = '\n'.join(report)
    print(full_report)
    if args.output:
        with open(args.output, 'w') as f:
            f.write(full_report)
        logger.info('Report written to %s', args.output)

    for df, title in [(df_rate,'Rate Errors'), (df_att,'Attitude Errors')]:
        for ax in ('roll','pitch','yaw'):
            plt.figure()
            plt.plot(df['time_s'], df[f'{ax}_err'], label=ax)
            plt.title(f'{title}: {ax.capitalize()}')
            plt.xlabel('Time (s)')
            plt.ylabel('Error')
            plt.legend()
            plt.tight_layout()
    plt.show()
