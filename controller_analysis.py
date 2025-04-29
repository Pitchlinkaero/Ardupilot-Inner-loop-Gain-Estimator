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

# --- Configuration Defaults ---
DEFAULT_TARGET_RATE_RMS = 5.0        # deg/s
DEFAULT_TARGET_ATT_ERR  = 2.0        # deg
DEFAULT_MAX_BUMP        = 1.20       # +20%
DEFAULT_TRIM_START      = 5.0        # seconds
DEFAULT_TRIM_END        = 5.0        # seconds
# Filter recommendation settings
PSDFREQ_MIN = 10     # Hz, minimum freq to consider in PSD
PSDFREQ_MAX = 200    # Hz, maximum freq to consider

# --- Setup Logger ---
logger = logging.getLogger('tuner')

# --- Helper: verify file exists ---
def verify_logfile(path):
    if not os.path.isfile(path):
        logger.error('Log file not found: %s', path)
        sys.exit(1)

# --- Gain & Filter Extraction ---
def extract_old_params(log_path):
    """
    Extract rate (ATC_RAT_*), attitude (ATC_ANG_*), and gyro filter (INS_GYRO_FILTER) parameters.
    Returns rate_gains, att_gains, and filter_old dict.
    """
    mlog = mavutil.mavlink_connection(log_path, dialect='ardupilotmega', robust_parsing=True)
    rate_gains = {ax: {} for ax in ('roll','pitch','yaw')}
    att_gains  = {ax: {} for ax in ('roll','pitch','yaw')}
    filter_old = {}

    # First PARM to get field names
    first = mlog.recv_match(type='PARM', blocking=True)
    if not first:
        logger.error('No PARM messages found')
        sys.exit(1)
    fields = first.get_fieldnames()
    id_field = next((f for f in fields if f.lower().replace('_','') in ('paramid','name')), None)
    val_field = next((f for f in fields if f.lower().replace('_','') in ('paramvalue','value')), None)
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
    # Include gyro filter parameter
    filter_param = 'INS_GYRO_FILTER'

    # Process all PARM messages
    msg = first
    while msg:
        name = getattr(msg, id_field, None)
        val  = getattr(msg, val_field, None)
        if name in pcm_rate:
            ax,k = pcm_rate[name]; rate_gains[ax][k] = val
        elif name in pcm_att:
            ax,k = pcm_att[name]; att_gains[ax][k] = val
        elif name == filter_param:
            filter_old['gyro_filter_hz'] = val
            logger.debug('Found old gyro filter: %s Hz', val)
        msg = mlog.recv_match(type='PARM', blocking=False)

    logger.info('Extracted rate gains: %s', rate_gains)
    logger.info('Extracted attitude gains: %s', att_gains)
    logger.info('Extracted old gyro filter: %s', filter_old)

    return rate_gains, att_gains, filter_old

# --- Parsing Helpers ---
def trim_df(df, start, end):
    tmax = df['time_s'].max()
    return df[(df['time_s']>=start) & (df['time_s']<=tmax-end)].reset_index(drop=True)

# --- Rate Analysis ---
def sniff_rate_fields(msg):
    names = msg.get_fieldnames(); des,act = {},{}
    for ax in ('roll','pitch','yaw'):
        des[ax] = next(f for f in names if f.lower().startswith(ax[0]+'des'))
        act[ax] = next(f for f in names if f.lower()==ax[0])
    return des, act

def parse_rate(log_path, trim_start, trim_end):
    m = mavutil.mavlink_connection(log_path, dialect='ardupilotmega', robust_parsing=True)
    first = m.recv_match(type='RATE', blocking=True)
    des,act = sniff_rate_fields(first)
    data = {'time_s': [], 'roll_err': [], 'pitch_err': [], 'yaw_err': []}
    t0 = first._timestamp
    data['time_s'].append(0.0)
    for ax in ('roll','pitch','yaw'):
        sp = getattr(first, des[ax]) * 180/np.pi
        ac = getattr(first, act[ax]) * 180/np.pi
        data[f'{ax}_err'].append(sp - ac)
    while True:
        msg = m.recv_match(type='RATE', blocking=False)
        if not msg: break
        t = msg._timestamp - t0; data['time_s'].append(t)
        for ax in ('roll','pitch','yaw'):
            sp = getattr(msg, des[ax]) * 180/np.pi
            ac = getattr(msg, act[ax]) * 180/np.pi
            data[f'{ax}_err'].append(sp - ac)
    df = pd.DataFrame(data)
    return trim_df(df, trim_start, trim_end)

# --- Attitude Analysis ---
def sniff_att_fields(msg):
    names = msg.get_fieldnames(); des,act = {},{}
    for ax in ('roll','pitch','yaw'):
        des_field = f'Des{ax.capitalize()}'
        des[ax] = next(f for f in names if f.lower()==des_field.lower())
        act[ax] = next(f for f in names if f.lower()==ax)
    return des, act

def parse_attitude(log_path, trim_start, trim_end):
    m = mavutil.mavlink_connection(log_path, dialect='ardupilotmega', robust_parsing=True)
    first = m.recv_match(type='ATT', blocking=True)
    des,act = sniff_att_fields(first)
    data = {'time_s': [], 'roll_err': [], 'pitch_err': [], 'yaw_err': []}
    t0 = first._timestamp
    while True:
        msg = m.recv_match(type='ATT', blocking=False)
        if not msg: break
        t = msg._timestamp - t0; data['time_s'].append(t)
        for ax in ('roll','pitch','yaw'):
            err = (getattr(msg, des[ax]) - getattr(msg, act[ax])) * 180/np.pi
            data[f'{ax}_err'].append(err)
    df = pd.DataFrame(data)
    return trim_df(df, trim_start, trim_end)

# --- IMU Parsing & Filter Recommendation ---
def parse_imu(log_path):
    m = mavutil.mavlink_connection(log_path, dialect='ardupilotmega', robust_parsing=True)
    first = m.recv_match(type='IMU', blocking=True)
    if not first:
        return pd.DataFrame()
    fields = first.get_fieldnames(); gyro_map={}
    for f in fields:
        lf = f.lower()
        if 'gyrx' in lf or lf.startswith('xgyro'): gyro_map['gyro_x']=f
        elif 'gyry' in lf or lf.startswith('ygyro'): gyro_map['gyro_y']=f
        elif 'gyrz' in lf or lf.startswith('zgyro'): gyro_map['gyro_z']=f
    data={'time_s':[],'gyro_x':[],'gyro_y':[],'gyro_z':[]}
    t0=first._timestamp; data['time_s'].append(0.0)
    for k in ('gyro_x','gyro_y','gyro_z'):
        val=getattr(first,gyro_map.get(k,''),0.0); data[k].append(val*180/np.pi)
    while True:
        msg=m.recv_match(type='IMU', blocking=False)
        if not msg: break
        t=msg._timestamp-t0; data['time_s'].append(t)
        for k in ('gyro_x','gyro_y','gyro_z'):
            val=getattr(msg,gyro_map.get(k,''),None)
            data[k].append(val*180/np.pi if val is not None else 0.0)
    return pd.DataFrame(data)

def recommend_filter(imu_df):
    if imu_df.empty:
        return {}
    fs=1/np.mean(np.diff(imu_df['time_s'])); rec={}
    for ax in ['gyro_x','gyro_y','gyro_z']:
        sig=imu_df[ax]-imu_df[ax].mean(); n=len(sig)
        freqs=np.fft.rfftfreq(n, d=1/fs); psd=np.abs(np.fft.rfft(sig))
        mask=(freqs>=PSDFREQ_MIN)&(freqs<=PSDFREQ_MAX)
        peak=freqs[mask][np.argmax(psd[mask])] if mask.any() else 0.0
        rec[ax]=peak*1.2
    return rec

# --- Summarize & Recommend Gains ---
def summarize(df):
    stats=[]
    for ax in ('roll_err','pitch_err','yaw_err'):
        e=df[ax]; stats.append({'axis':ax.replace('_err',''), 'mean':e.mean(), 'rms':np.sqrt((e**2).mean()), 'max':e.abs().max()})
    return pd.DataFrame(stats)

def recommend_rate(stats, old, target_rms, max_bump):
    rec={}
    for _,r in stats.iterrows():
        ax=r['axis']; scale=r['rms']/target_rms
        P_old,I_old,D_old=old[ax]['P'],old[ax]['I'],old[ax]['D']
        P_new=min(P_old*scale,P_old*max_bump)
        rec[ax]={'P_old':P_old,'I_old':I_old,'D_old':D_old,'P_new':P_new,'I_new':P_new/10,'D_new':P_new/100,'scale':scale}
    return pd.DataFrame.from_dict(rec,orient='index')

def recommend_att(stats, old, target_err, max_bump):
    rec={}
    for _,r in stats.iterrows():
        ax=r['axis']; scale=r['max']/target_err; P_old=old[ax]['P']
        P_new=min(P_old*scale,P_old*max_bump)
        rec[ax]={'P_old':P_old,'P_new':P_new,'scale':scale}
    return pd.DataFrame.from_dict(rec,orient='index')

# --- Main Execution ---
if __name__=='__main__':
    parser=argparse.ArgumentParser(description='Controller log tuner with filter')
    parser.add_argument('logfile',help='ArduPilot .bin log')
    parser.add_argument('-v','--verbose',action='store_true',default=False,help='Verbose on')
    parser.add_argument('-f','--filter',action='store_true',default=True,help='Filter rec on')
    parser.add_argument('-o','--output',help='Output report file',default=None)
    parser.add_argument('--rate-rms',type=float,default=DEFAULT_TARGET_RATE_RMS)
    parser.add_argument('--att-err',type=float,default=DEFAULT_TARGET_ATT_ERR)
    parser.add_argument('--bump',type=float,default=DEFAULT_MAX_BUMP)
    parser.add_argument('--trim-start',type=float,default=DEFAULT_TRIM_START)
    parser.add_argument('--trim-end',type=float,default=DEFAULT_TRIM_END)
    args=parser.parse_args()

    level=logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level,format='%(asctime)s %(levelname)s %(message)s')

    verify_logfile(args.logfile)
    logger.info('Starting analysis on %s',args.logfile)

    rate_old, att_old, filter_old = extract_old_params(args.logfile)

    df_rate=parse_rate(args.logfile,args.trim_start,args.trim_end)
    rate_stats=summarize(df_rate); rate_rec=recommend_rate(rate_stats,rate_old,args.rate_rms,args.bump)

    df_att=parse_attitude(args.logfile,args.trim_start,args.trim_end)
    att_stats=summarize(df_att); att_rec=recommend_att(att_stats,att_old,args.att_err,args.bump)

    report=[
        '=== Old Gyro Filter Setting ===', f"gyro_filter_hz: {filter_old.get('gyro_filter_hz','N/A')}",
        '=== Rate Error Summary ===', rate_stats.to_string(index=False),
        '=== Rate Gain Recommendations ===', rate_rec.to_string(),
        '=== Attitude Error Summary ===', att_stats.to_string(index=False),
        '=== Attitude Gain Recommendations ===', att_rec.to_string()
    ]

    if args.filter:
        imu_df=parse_imu(args.logfile)
        filt_rec=recommend_filter(imu_df)
        report+=['=== Gyro Filter Recommendations (Hz) ===',pd.Series(filt_rec).to_string()]

    full_report='\n'.join(report)
    print(full_report)

    if args.output:
        with open(args.output,'w') as f: f.write(full_report)
        logger.info('Report written to %s',args.output)

    for df,title in [(df_rate,'Rate Errors'),(df_att,'Attitude Errors')]:
        for ax in ('roll','pitch','yaw'):
            plt.figure(); plt.plot(df['time_s'],df[f'{ax}_err'],label=f'{ax}'); plt.title(f'{title}: {ax}'); plt.xlabel('Time(s)'); plt.ylabel('Error'); plt.legend();plt.tight_layout()
    plt.show()
