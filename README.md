# Ardupilot-Inner-loop-Gain-Estimator
This script automates the analysis and tuning of ArduPilot rate and attitude controllers (and recommends gyro filter settings) based on flight log data.

---

## Features

- **Extract existing parameters** from the log:  
  - Rate controller gains: `ATC_RAT_<axis>_{P,I,D}`  
  - Attitude controller gains: `ATC_ANG_<axis>_P`  
  - Gyro filter cutoff: `INS_GYRO_FILTER`

- **Rate loop analysis**  
  - Compute error: `error = setpoint_rate - actual_rate` in deg/s  
  - Trim first/last data to avoid transients  
  - Calculate mean, RMS, and peak errors  
  - Recommend new PID gains by scaling old gains to hit a target RMS error (default 5 deg/s), capped at +20 %

- **Attitude loop analysis**  
  - Compute error: `error = setpoint_angle - actual_angle` in deg  
  - Trim data similarly  
  - Calculate mean, RMS, and peak errors  
  - Recommend new P gains to meet a target max attitude error (default 2 deg), capped at +20 %

- **Gyro filter recommendation** (optional)  
  - Parse raw IMU gyro data (X/Y/Z)  
  - Compute FFT and power spectrum in a noise band (10–200 Hz)  
  - Suggest a filter cutoff at ~1.2 × dominant noise frequency

- **Reporting & plots**  
  - Console report of old vs. recommended settings  
  - Optional output to text file (`-o` flag)  
  - Plots of error vs. time using matplotlib

---

## Installation

1. Install dependencies:
   ```bash
   pip install pymavlink pandas matplotlib numpy
   ```

---

## Usage

```bash
python rate_controller_analysis.py <logfile.bin> [options]
```

### Options

| Flag                  | Description                                       | Default    |
|-----------------------|---------------------------------------------------|------------|
| `-v`, `--verbose`     | Enable detailed logging                           | on         |
| `-f`, `--filter`      | Recommend gyro filter cutoff                      | on         |
| `-o FILE`, `--output` | Write report to FILE                              | (stdout)   |
| `--rate-rms X`        | Target rate RMS error (deg/s)                     | 5.0        |
| `--att-err X`         | Target attitude max error (deg)                   | 2.0        |
| `--bump X`            | Max gain bump factor                              | 1.2        |
| `--trim-start X`      | Seconds to trim at beginning                      | 5.0        |
| `--trim-end X`        | Seconds to trim at end                            | 5.0        |

**Example:**
```bash
python rate_controller_analysis.py logfile.bin -o report.txt
```

---

## Mathematical Details

### Error Computation

For each axis (roll, pitch, yaw) the script computes:  

```text
rate_error(t)    = rate_setpoint(t)    - rate_actual(t)    # in deg/s
attitude_error(t) = angle_setpoint(t)   - angle_actual(t)   # in deg
```

Data is trimmed by removing the first and last _X_ seconds (configurable) to avoid takeoff/landing effects.

### Summary Statistics

Over the trimmed time series of _N_ samples:

- **Mean error**:  
  `mean_error = (1/N) * sum_i(e_i)`

- **RMS error**:  
  `rms_error = sqrt((1/N) * sum_i(e_i^2))`

- **Peak error**:  
  `peak_error = max_i |e_i|`

### Gain Recommendation

New gains are computed by scaling the old gains proportionally to the measured error vs. target, then capping the increase:

```text
P_new = min(
    P_old * (measured_error / target_error),
    P_old * max_bump_factor
)
```

- **Rate loop** uses measured RMS vs. target RMS  
- **Attitude loop** uses measured peak error vs. target max  

The script also sets inner-loop I and D gains by fixed ratios of the new P:
```text
I_new = P_new * 0.1
D_new = P_new * 0.01
```

### Filter Recommendation

To suggest a gyro filter cutoff:  
1. Compute the sampling rate `fs` from IMU timestamps.  
2. Perform an FFT of the zero-mean gyro signal to get frequencies and power.  
3. Identify the dominant noise frequency `f_peak` within 10–200 Hz.  
4. Recommend:  
   ```text
   cutoff = 1.2 * f_peak
   ```

---

## License

MIT © Pitchlink, LLC 

