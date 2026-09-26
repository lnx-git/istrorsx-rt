# `odom_analyze.py` — what its output means

`odom_analyze.py` reads a finished run's logs and compares the control board's speed data with GPS on
straight driving. GPS is taken as the reference. The goal is to learn what the board's numbers mean in
absolute terms: how many metres one encoder pulse is, and what speed a motor setpoint gives.

This file explains every value the script writes, so its output can be read without opening the code.

```bash
python3 script/log/odom_analyze.py archive/<test_dir>                    # log-line output
python3 script/log/odom_analyze.py -q archive/<test_dir>                 # summaries only
python3 script/log/odom_analyze.py --csv odom_analyze_segments.csv \
        --fixes-csv odom_analyze_fixes.csv archive/<test_dir>            # + the two CSV files
```

The input is a `logout/` directory, an archived run (`archive/<test_dir>/logout_*`), or any directory above
several runs. Rotated and gzipped logs are handled. The logs must contain the DEBUG lines
`ctrlBoard_writeData()` and `gps_writeData()`, which `conf/log4cxx.xml` logs by default.

Saved results: `doc/<date>_test_<park>/odom_analyze.txt`, `odom_analyze_segments.csv`,
`odom_analyze_fixes.csv`.

---

## 1. Raw signals the script uses

| signal | source line in the log | meaning |
|---|---|---|
| `ctrlb_velocity` | `ctrlBoard_writeData(): state=…` (~145×/s) | motor setpoint the control board reports. `335` = stop (`VEL_ZERO`), higher = forward, lower = reverse. `347` means +12. |
| `ctrlb_ircv` | same line | encoder pulses since the previous telemetry frame (usually 0, 1 or 2). Summing them gives the pulse count. |
| `ircv500` | same line | pulses in the last 500 ms, interpolated by the ROS side (`ctrlboard.cpp` `ircth_get()`), so it can be fractional. |
| `ctrlb_angle` | same line | steering servo value. `334` = straight (`SA_STRAIGHT`); one unit ≈ 0.75° of steering. |
| `state` | same line | board state: `1` STOP, `2` FWD, `3` BCK, `5`/`6`/`7` obstacle, `8` emergency. |
| IMU yaw | `ctrlBoard_writeData(): euler_x=…` (~8×/s) | BNO055 compass heading, degrees, not north-aligned. |
| GPS | `istro::gps_writeData(): fix=1, …` (1×/s) | latitude, longitude, speed (m/s), course (degrees). Only fixes with `fix=1` are used. |
| calibration window | `calibProcess2(): msg="calibration in progress..."` | stretches `planner_node` itself picked as possibly straight for its compass calibration. |

**The code's own assumption**: `planner_node`'s `updateXY()` computes the robot speed as
`ircv500 × 0.03` m/s. Because `ircv500` counts half a second, that is **0.015 m per pulse**, although the
comment in the code says "one impulse = 3 centimetres".

---

## 2. What a segment is

Every result is computed over a **segment**: a stretch of straight driving at a constant setpoint. Two
kinds are reported side by side (`source`):

* **`source="straight"`** — found by the script itself. All of these must hold:
  * the same `ctrlb_velocity`, not `335`, for the whole stretch;
  * board `state` `2` (FWD) or `3` (BCK) the whole time;
  * steering within `--angle-tol` (default 20 units ≈ 15°) of straight;
  * no gap over 500 ms in the telemetry.
  The first `--settle` s (default 1.5) are dropped, so acceleration and the lag of `ircv500` do not count.
  The rest must last at least `--min-seg` s (default 5).
* **`source="calib"`** — `planner_node`'s own calibration windows, with the first `--settle` s dropped and
  at least `--min-calib` s left (default 4). The planner also logs attempts it cancels a moment later, so
  these windows are checked the same way; rejected ones are only counted (`calib_windows_rejected`).

Both kinds must then pass the same straightness checks:
* at least 3 GPS fixes inside;
* IMU yaw spread within `--yaw-tol` degrees (default 5);
* GPS course spread within 15° (only fixes at ≥ 0.3 m/s count).

"Spread" is the difference between the largest and smallest value around their circular mean.

---

## 3. Two encoder modes (`mode`)

On the 2026-09-10/12 logs the encoder does not have one scale. At the same GPS speed it reports either:

* **`mode="low"`** — ~35-45 pulses per m/s, i.e. **~0.02-0.03 m per pulse**; or
* **`mode="high"`** — ~200-250 pulses per m/s, i.e. **~0.004-0.008 m per pulse**.

We observed these two regimes while driving: for an unknown reason the control board at times sends a much
higher number of pulses for the same speed. Each mode lasts from seconds to minutes. A median
over both modes describes neither, so the script classifies every segment and every GPS fix. The split is
`0.012` m per pulse (`MODE_SPLIT_M_PER_PULSE`), between the two groups.

* A **segment's** mode uses `m_per_pulse_disp`, or `m_per_pulse_speed` if the displacement is not available.
* A **fix's** mode uses `gps_speed / pulses_1s`, only when GPS speed is at least 0.3 m/s.
* **Empty `mode`** — nothing to classify (no pulses or no GPS speed).

---

## 4. Log-line output (`odom_analyze.txt`)

Every line has the format of `istro_rt2026.log`:

```
<date> <time> INFO  [analyze] odomAnalyze::<kind>(): run="<run>", msg="<message>", <field>=<value>, …
```

Kinds of line:

| kind | message | timestamp | when |
|---|---|---|---|
| `segment()` | `segment` | segment start | one per accepted segment (not with `-q`) |
| `setpoint()` | `per setpoint` | last telemetry frame of the run, or time of the analysis for totals | one per setpoint value, per source and mode |
| `summary()` | `run summary` | last telemetry frame of the run | one per source, for each run |
| `summary()` | `total summary per mode` | time of the analysis | one per source and mode, only if more than one run was analysed |
| `summary()` | `total summary` | time of the analysis | one per source, both modes together, only if more than one run |

### 4.1 `segment()` fields

| field | unit | meaning |
|---|---|---|
| `source` | — | `straight` or `calib`, see §2 |
| `t_end` | time | end of the segment; the start is the line's timestamp |
| `dur` | s | segment length |
| `velocity` | setpoint | most frequent `ctrlb_velocity` in the segment |
| `vel_offset` | setpoint units | `velocity − 335`. Positive = forward. |
| `angle_mean` | servo units | mean steering value (`334` = straight) |
| `pulses` | pulses | sum of `ctrlb_ircv` over the segment |
| `pulse_rate` | pulses/s | `pulses / dur` |
| `ircv500_mean` | pulses per 0.5 s | mean `ircv500`. About `pulse_rate / 2` if the telemetry is consistent. |
| `code_speed` | m/s | the speed `updateXY()` would compute: `ircv500_mean × 0.03` |
| `gps_fixes` | count | GPS fixes inside the segment |
| `gps_speed_mean` | m/s | mean GPS speed over those fixes |
| `gps_disp` | m | straight-line distance between the first and the last fix in the segment |
| `gps_disp_speed` | m/s | `gps_disp` divided by the time between those two fixes |
| `pulses_disp` | pulses | encoder pulses between the first and the last fix — the pulses matching `gps_disp` |
| `m_per_pulse_speed` | m | `gps_speed_mean / pulse_rate` — metres per pulse from GPS speed |
| `m_per_pulse_disp` | m | `gps_disp / pulses_disp` — metres per pulse from GPS displacement. **The more reliable of the two.** |
| `course_spread` | degrees | spread of GPS course over fixes at ≥ 0.3 m/s |
| `yaw_spread` | degrees | spread of the IMU yaw |
| `calib_overlap` | 0-1 | share of the segment covered by planner calibration windows. `1.00` for every `calib` segment. |
| `mode` | — | `low` / `high`, see §3 |

### 4.2 `setpoint()` fields

Medians over the segments with one setpoint value, one source and one mode:

| field | meaning |
|---|---|
| `source`, `mode`, `velocity`, `vel_offset` | the group |
| `segments`, `total_s` | how many segments and their total length in s |
| `pulse_rate_median` | median pulses per second |
| `ircv500_median` | median of the segments' mean `ircv500` |
| `code_speed_median` | median speed as `updateXY()` computes it, m/s |
| `gps_speed_median` | median GPS speed, m/s |
| `gps_disp_speed_median` | median speed from GPS displacement, m/s |
| `m_per_pulse_disp_median` | median metres per pulse from GPS displacement |

### 4.3 `summary()` fields

| field | meaning |
|---|---|
| `source` (and `mode` in "per mode" lines) | the group |
| `segments`, `total_s` | how many segments and their total length in s |
| `mode_low`, `mode_high` | how many of those segments are in each mode |
| `m_per_pulse_disp_median` | median of the segments' `m_per_pulse_disp` |
| `m_per_pulse_disp_pooled` | all GPS displacement divided by all matching pulses — longer segments weigh more |
| `m_per_pulse_speed_median` | median of the segments' `m_per_pulse_speed` |
| `code_m_per_pulse` | what the code assumes: `0.0150` |
| `gps_disp_speed_to_code_speed_median` | median of `gps_disp_speed / code_speed`. **Above 1: the local position under-reports distance** (e.g. 1.6 = the robot drove 60 % further than it thinks). Below 1: it over-reports. |
| `corr_pulse_rate_gps_disp_speed` | Pearson correlation between pulse rate and GPS speed over segments (−1…1). Near 1 means the encoder tracks real speed. |
| `corr_vel_offset_gps_disp_speed` | correlation between the setpoint and GPS speed |
| `corr_vel_offset_pulse_rate` | correlation between the setpoint and the pulse rate |
| `calib_windows_rejected` | (`calib` only) planner windows that failed the straightness checks |
| `note` | why a run gave nothing, e.g. no telemetry or no GPS fix |

A correlation is empty when there are fewer than 3 segments or the values do not vary — for example all
segments with the same setpoint.

---

## 5. `odom_analyze_segments.csv`

One row per accepted segment, all runs. It has the columns of §4.1 plus these:

| column | meaning |
|---|---|
| `run` | run directory name |
| `t_start`, `t_end` | segment start and end, full timestamp |
| `gps_speed_median` | median GPS speed in the segment, m/s |
| `gps_disp_dt` | s between the first and the last fix |
| `course_mean` | circular mean GPS course, degrees |
| `yaw_mean` | circular mean IMU yaw, degrees |

Numbers have 5 decimals, and an empty cell means not available.

**Both CSV files use the Slovak locale**, so they open directly in a Slovak spreadsheet: columns are separated by `;` and numbers use a decimal comma (`0,02430`). Timestamps keep their own comma before the milliseconds (`2026-09-12 11:29:37,403`). The log-line output in `odom_analyze.txt` keeps decimal points.

## 6. `odom_analyze_fixes.csv`

One row per GPS fix while the setpoint is not `335` — whether driving straight or not. This is the raw
material for further analysis, e.g. scatter plots of GPS speed against pulses.

| column | meaning |
|---|---|
| `run`, `t` | run and the fix's log time |
| `gps_speed` | GPS speed, m/s |
| `gps_course` | GPS course, degrees |
| `mode` | `low`/`high` from `gps_speed / pulses_1s` (empty below 0.3 m/s or with no pulses) |
| `pulses_1s` | encoder pulses in the 1 s before the fix — directly comparable with `gps_speed` |
| `ircv500` | `ircv500` of the last telemetry frame before the fix |
| `velocity` | setpoint of that frame |
| `state` | board state of that frame |
| `angle` | steering value of that frame |
| `yaw_spread_1s` | IMU yaw spread in the 1 s before the fix, degrees. **Small = the robot was driving straight.** |
| `in_calib` | `1` if more than half of that second was inside a planner calibration window |

---

## 7. Reading the results

* **Use `m_per_pulse_disp` / `…_pooled`, per mode.**
  * GPS speed over-reads at low speed, because its noise only ever adds to it, so `m_per_pulse_speed`
    comes out a little high.
  * GPS displacement over several seconds of straight driving does not have that bias.
* **Compare with `code_m_per_pulse` (0.015).** In the 2026-09-10/12 logs, `low` mode gives ~0.023-0.027
  (the local position under-reports distance ~1.6×) and `high` mode ~0.006-0.008 (it over-reports ~2×).
* **Check the correlation per mode.** Inside one mode, pulse rate and GPS speed correlate at ~0.6-0.9;
  mixed together they drop to ~0.1-0.5.
* **The numbers are preliminary**: 6-13 segments per test day, one consumer GPS at 1 Hz. A tape-measured
  straight is still the decisive test for the true scale, and it should record which mode the encoder was in.
