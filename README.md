# Ardupilot-Inner-loop-Gain-Estimator

This script automates the analysis and tuning of ArduPilot rate and attitude controllers (and recommends gyro filter settings and actuator delay/FF estimation) based on flight log data.

---

## Features

- **Extract existing parameters** from the log:  
  - Rate controller gains: `ATC_RAT_<axis>_{P,I,D}`  
  - Attitude controller gains: `ATC_ANG_<axis>_P`  
  - Gyro filter cutoff: `INS_GYRO_FILTER`  
  - Rate feed‑forward gains: `ATC_RAT_<axis>_FF`

- **Rate loop analysis**  
  - Compute error: `error = setpoint_rate - actual_rate` (deg/s)  
  - Trim first/last seconds to avoid transients  
  - Calculate mean, RMS, and peak errors  
  - Recommend new PID gains by scaling old gains to hit a target RMS error (default 5 deg/s), capped at +20 %

- **Attitude loop analysis**  
  - Compute error: `error = setpoint_angle - actual_angle` (deg)  
  - Trim data similarly  
  - Calculate mean, RMS, and peak errors  
  - Recommend new P gains to meet a target max attitude error (default 2 deg), capped at +20 %

- **Gyro filter recommendation** (optional)  
  - Parse raw IMU gyro data (X/Y/Z)  
  - Compute FFT and power spectrum in 10–200 Hz  
  - Suggest cutoff at ~1.2 × dominant noise frequency

- **Actuator delay estimation** (optional)  
  - Cross‑correlate setpoint and actual response to estimate delay  
  - Time‐shift data and report median delay

- **Feed‑forward estimation**  
  - Perform least‐squares fit of controller output vs. setpoint  
  - Suggest updated `ATC_RAT_<axis>_FF`

- **Reporting & plots**  
  - Console report of old vs. recommended settings  
  - Optional output to text file (`-o`)  
  - Progress bars during parsing (requires `tqdm`)  
  - Matplotlib plots of error vs. time

---

## Installation

```bash
pip install pymavlink pandas numpy matplotlib tqdm
```

---

## Usage

```bash
python rate_controller_analysis.py <logfile.bin> [options]
```

### Options

| Flag                  | Description                                       | Default    |
|-----------------------|---------------------------------------------------|------------|
| `-v`, `--verbose`     | Enable detailed logging                           | off        |
| `-f`, `--filter`      | Recommend gyro filter cutoff                      | on         |
| `-d`, `--delay`       | Estimate & apply actuator delay                   | on         |
| `-o FILE`, `--output` | Write report to FILE                              | (stdout)   |
| `--rate-rms X`        | Target rate RMS error (deg/s)                     | 5.0        |
| `--att-err X`         | Target attitude max error (deg)                   | 2.0        |
| `--bump X`            | Max gain bump factor                              | 1.2        |
| `--trim-start X`      | Seconds to trim at beginning                      | 5.0        |
| `--trim-end X`        | Seconds to trim at end                            | 5.0        |

**Example:**

```bash
python rate_controller_analysis.py 20250315_N910FH_F3_AltHold.bin -o report.txt
```

---

## Mathematical Details

### Error Computation

For each axis:

```text
rate_error(t)     = rate_setpoint(t)   - rate_actual(t)    # deg/s
attitude_error(t) = angle_setpoint(t)  - angle_actual(t)   # deg
```

Data is trimmed by removing the first and last _X_ seconds to avoid takeoff/landing transients.

### Summary Statistics

Over _N_ samples:

- Mean error:  
  `mean = (1/N) * Σ e_i`

- RMS error:  
  `rms = sqrt((1/N) * Σ e_i^2)`

- Peak error:  
  `peak = max |e_i|`

### Gain Recommendation

```text
P_new = min(
  P_old * (measured_error / target_error),
  P_old * max_bump_factor
)
I_new = P_new * 0.1
D_new = P_new * 0.01
```

- **Rate loop** uses measured RMS vs. `--rate-rms`  
- **Attitude loop** uses measured peak vs. `--att-err`

### Filter Recommendation

```text
cutoff = 1.2 * f_peak
```
Where `f_peak` is the dominant noise frequency (10–200 Hz) from the FFT of zero‑mean gyro data.

### Delay Estimation

```python
# assume fs = sampling rate
tau = estimate_delay(setpoint, response, fs)
```
Use cross‑correlation to find the lag of maximum correlation.

### Feed‑Forward Estimation

Fit controller output vs. setpoint via least‑squares:
```text
FF_est = (Σ y_i x_i) / (Σ x_i^2)
```
where `y_i` is controller output (ROut/POut/YOut) and `x_i` is setpoint.

---

## License

MIT © Pitchlink, LLC

