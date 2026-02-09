# L1 Delta-Phi Feedback Experiment

This branch keeps Ruben's adaptive lateral-acceleration inversion and layers a minimal incremental feedback term on top.

## Autotest Script (run first)

```bash
cd /Users/nathanielmailhot/Documents/ArduDev/ardupilot
./waf plane
Tools/autotest/run_l1_indi_abtest.sh --instance 1
```

Equivalent direct call:

```bash
python3 /Users/nathanielmailhot/Documents/ArduDev/ardupilot/Tools/autotest/run_l1_indi_abtest.py \
  --vehicle-binary /Users/nathanielmailhot/Documents/ArduDev/ardupilot/build/sitl/bin/arduplane \
  --mission /Users/nathanielmailhot/Documents/ArduDev/ardupilot/Tools/autotest/Generic_Missions/CMAC-loiter-unlim.txt \
  --instance 1 \
  --speedup 5 \
  --sim-speedup-param 5 \
  --off-duration 130 \
  --on-duration 130
```

## Latest A/B results (from `logs/00000017.BIN`)

Conditions:
- Loiter radius command: `120 m`
- Wind: `18 m/s` from `270 deg` (`SIM_WIND_TC=0.8`, `SIM_WIND_TURB=1.5`)
- Segment lengths: `130.34 s` OFF and `130.34 s` ON

Metrics:
- `MAE`: `9.02 m` (OFF) -> `4.71 m` (ON), `-47.8%`
- `RMSE`: `15.08 m` (OFF) -> `8.33 m` (ON), `-44.8%`
- `P95 |error|`: `33.54 m` (OFF) -> `20.90 m` (ON), `-37.7%`
- `Max outward error`: `44.47 m` (OFF) -> `24.28 m` (ON), `-45.4%`
- `Within +/-10 m`: `70.8%` (OFF) -> `80.7%` (ON)

## Controller form

Base (adaptive inversion):

`phi_ff = atan(a_dem / (g * cos(theta) * k_hat))`

Added incremental correction:

`phi_cmd = phi_ff + delta_phi`

`delta_phi = (Kp*e + Ki*integral(e) + Kd*dedt) / (g * cos(theta) * k_hat * sec(phi_ff)^2)`

`e = a_dem - a_meas_filt`

Notes:
- `k_hat` is still adapted by `NAVL1_LACC_K_TC`.
- `P/I/D` are fixed params, not auto-tuned.
- `NAVL1_LACC_FB_TC` low-pass filters accel error.

## Recommended SITL settings

Use these as a starting point:

```
NAVL1_LACC_K_TC  = 4.0
NAVL1_LACC_FB_TC = 0.35
NAVL1_LACC_FB_P  = 0.10
NAVL1_LACC_FB_I  = 0.015
NAVL1_LACC_FB_D  = 0.04
```

Mission file used in this experiment:

`/Users/nathanielmailhot/Documents/ArduDev/ardupilot/Tools/autotest/Generic_Missions/CMAC-loiter-unlim.txt`

## Quick start

Build:

```bash
cd /Users/nathanielmailhot/Documents/ArduDev/ardupilot
./waf plane
```

Run the automated A/B test (no manual MAVProxy steps required):

```bash
python3 /Users/nathanielmailhot/Documents/ArduDev/ardupilot/Tools/autotest/run_l1_indi_abtest.py \
  --vehicle-binary /Users/nathanielmailhot/Documents/ArduDev/ardupilot/build/sitl/bin/arduplane \
  --mission /Users/nathanielmailhot/Documents/ArduDev/ardupilot/Tools/autotest/Generic_Missions/CMAC-loiter-unlim.txt \
  --instance 1 \
  --speedup 5 \
  --sim-speedup-param 5 \
  --off-duration 130 \
  --on-duration 130
```

This automated run:
- starts in `AUTO`
- waits for loiter at waypoint 2
- runs feedback OFF segment under wind
- enables feedback and runs ON segment
- switches to `RTL`
- writes OFF/ON CSV plus a summary JSON

## Artifacts

By default outputs go to:

`/Users/nathanielmailhot/Documents/ArduDev/buildlogs/l1_indi_ab/`

Files:
- `l1_indi_ab_full.bin`: full DataFlash log copy
- `l1_indi_ab_full.tlog`: full TLOG copy (if available)
- `l1_indi_ab_off.csv`: OFF-segment radius/error data
- `l1_indi_ab_on.csv`: ON-segment radius/error data
- `l1_indi_ab_summary.json`: metrics and percentage deltas

Metrics include:
- `mae_m`
- `rmse_m`
- `p95_abs_error_m`
- `max_outward_error_m`
- `within_10m_percent`
