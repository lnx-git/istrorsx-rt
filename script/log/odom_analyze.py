#!/usr/bin/env python3
"""Relate control-board speed data to GPS speed on straight driving, from istro_<EVENT_TAG> logs.

Usage:
    odom_analyze.py LOGDIR [LOGDIR ...]                  # logout_* dirs, or any dir above them
    odom_analyze.py --csv odom_analyze_segments.csv LOGDIR          # + one CSV row per segment
    odom_analyze.py --fixes-csv odom_analyze_fixes.csv LOGDIR       # + one CSV row per GPS fix while moving

Saved next to odom_analyze.txt in doc/<date>_test_<park>/ under these names, so "odom_analyze" prefixes
every output of this script.
    odom_analyze.py -q LOGDIR                            # summaries only

Output is in the same line format as istro_<EVENT_TAG>.log. GPS speed and GPS displacement are taken as the
reference; the question is what the control board's numbers mean in absolute terms:

  ctrlb_velocity  motor setpoint the board reports (VEL_ZERO=335; 347 = +12)
  ctrlb_ircv      encoder pulses since the previous telemetry frame (~145 frames/s)
  ircv500         pulses in the last 500 ms (ctrlboard.cpp ircth_get(), interpolated)

planner_node's updateXY() uses speed = ircv500 * 0.03 m/s, i.e. 0.015 m per pulse, although its comment
says "one impulse = 3 centimetres". This script measures metres per pulse against GPS.

Two kinds of segment, both reported:
  source="straight"  own detection: constant forward/backward setpoint, board state FWD/BCK, steering within
                     --angle-tol of SA_STRAIGHT, IMU yaw spread <= --yaw-tol, GPS course spread <= 15 deg,
                     >= --min-seg s after dropping the first --settle s, >= 3 GPS fixes
  source="calib"     planner_node's own calibration windows (calibProcess2 "calibration in progress...").
                     The planner logs those lines also for attempts it later cancels, so the windows get the
                     same yaw/course/fix checks (after --settle); rejected windows are counted in the summary

Per segment: pulses and pulse rate, mean ircv500, the speed updateXY() would compute, mean/median GPS speed,
GPS displacement between the first and last fix and the pulses in between, metres per pulse from GPS speed
and from GPS displacement. Summaries add per-setpoint medians and correlations.
GPS speed over-reads at low speed (noise only adds); the displacement-based figures do not have that bias.

Every output field and CSV column is explained in script/log/odom_analyze.md. The CSV files use the Slovak
locale: ';' between columns and a decimal comma (the log-line output keeps decimal points).

Two encoder regimes: on the 2026-09-10/12 logs, at the same GPS speed the encoder reports either ~35-45
pulses per m/s (~0.02-0.03 m per pulse) or ~200-250 (~0.004-0.005), each lasting from seconds to minutes. A
median across both means nothing, so every segment and fix gets mode="low"/"high" (split at
MODE_SPLIT_M_PER_PULSE) and the summaries are also given per mode.
"""

import argparse
import bisect
import csv
import datetime
import gzip
import math
import os
import re
import statistics
import sys

LOG_NAME = re.compile(r"^istro_[a-z]+\d+(\.\d\d_\d\d)?\.log(\.gz)?$")

VEL_ZERO = 335
SA_STRAIGHT = 334
STATE_FWD, STATE_BCK = 2, 3
CODE_SPEED_COEF = 0.03             # planner_node.cpp UPDATEXY_SPEED_COEF, applied to ircv500
CODE_M_PER_PULSE = CODE_SPEED_COEF / 2.0   # ircv500 counts 0.5 s, so 0.015 m per pulse
COMMENT_M_PER_PULSE = 0.03         # what UPDATEXY_SPEED_COEF's comment claims
COURSE_MAX_SPREAD = 15.0
GPS_MOVING_SPEED = 0.3
CALIB_GAP_S = 0.3
MODE_SPLIT_M_PER_PULSE = 0.012     # between the two regimes seen in the logs, see the docstring

RE_SERVO = re.compile(r'state=(\d+), ctrlb_ircv=(-?\d+), ircv500=([\d.]+), ctrlb_angle=(\d+), ctrlb_velocity=(-?\d+)')
RE_EULER = re.compile(r'euler_x=([-\d.]+)')
RE_GPS = re.compile(r'fix=1, latitude=([-\d.]+), longitude=([-\d.]+), .*speed=([-\d.]+), course=([-\d.]+)')

_day_cache = {}


def ts_ms(line):
    d = line[:10]
    base = _day_cache.get(d)
    if base is None:
        base = datetime.date.fromisoformat(d).toordinal() * 86400000
        _day_cache[d] = base
    return base + int(line[11:13]) * 3600000 + int(line[14:16]) * 60000 + int(line[17:19]) * 1000 + int(line[20:23])


def fmt_ts(ms):
    day, rest = divmod(int(ms), 86400000)
    h, rest = divmod(rest, 3600000)
    m, rest = divmod(rest, 60000)
    s, milli = divmod(rest, 1000)
    return f"{datetime.date.fromordinal(day).isoformat()} {h:02d}:{m:02d}:{s:02d},{milli:03d}"


def now_ms():
    n = datetime.datetime.now()
    return n.date().toordinal() * 86400000 + ((n.hour * 60 + n.minute) * 60 + n.second) * 1000 + n.microsecond // 1000


def norm(a):
    return (a + 180.0) % 360.0 - 180.0


def cmean(vals):
    s = sum(math.sin(math.radians(v)) for v in vals)
    c = sum(math.cos(math.radians(v)) for v in vals)
    return math.degrees(math.atan2(s, c)) % 360.0


def cspread(vals):
    if not vals:
        return 0.0
    m = cmean(vals)
    d = [norm(v - m) for v in vals]
    return max(d) - min(d)


def haversine(lat1, lon1, lat2, lon2):
    la1, lo1, la2, lo2 = map(math.radians, (lat1, lon1, lat2, lon2))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * 6371000.0 * math.asin(math.sqrt(h))


def pearson(xs, ys):
    if len(xs) < 3:
        return None
    mx, my = statistics.mean(xs), statistics.mean(ys)
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    sy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if sx == 0 or sy == 0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (sx * sy)


def log_files(run_dir):
    files = [os.path.join(run_dir, f) for f in os.listdir(run_dir) if LOG_NAME.match(f)]

    def first_ts(path):
        opener = gzip.open if path.endswith(".gz") else open
        try:
            with opener(path, "rt", errors="replace") as fh:
                for line in fh:
                    if len(line) > 23 and line[4] == "-" and line[13] == ":":
                        return ts_ms(line)
        except OSError:
            pass
        return float("inf")
    return sorted(files, key=first_ts)


def run_dirs(paths):
    for p in paths:
        if os.path.isfile(p):
            yield os.path.dirname(p) or "."
            continue
        found = False
        for root, dirs, files in os.walk(p):
            dirs.sort()
            if any(LOG_NAME.match(f) for f in files):
                found = True
                yield root
        if not found:
            print(f"odom_analyze.py: no istro_<tag> logs under {p}", file=sys.stderr)


def emit(level, t, method, msg, run, fields):
    parts = [f'run="{run}"', f'msg="{msg}"']
    for k, v in fields.items():
        parts.append(f'{k}="{v}"' if isinstance(v, str) else f"{k}={v}")
    print(f"{fmt_ts(t)} {level:<5} [analyze] odomAnalyze::{method}(): " + ", ".join(parts))


def f2(v, nd=3):
    return "" if v is None else f"{v:.{nd}f}"


class Run:
    """All samples of one run, time-ordered, with prefix sums for pulse counts."""

    def __init__(self, name):
        self.name = name
        self.servo_t, self.servo = [], []      # (state, ircv, ircv500, angle, velocity)
        self.euler_t, self.euler = [], []
        self.gps_t, self.gps = [], []          # (lat, lon, speed, course)
        self.calib = []                        # [t0, t1] windows
        self.cum = [0]
        self.last_t = 0

    def load(self, run_dir):
        cal_last = None
        for path in log_files(run_dir):
            opener = gzip.open if path.endswith(".gz") else open
            with opener(path, "rt", errors="replace") as fh:
                for line in fh:
                    if len(line) < 24 or line[4] != "-" or line[13] != ":":
                        continue
                    if "ctrlBoard_writeData(): state=" in line:
                        m = RE_SERVO.search(line)
                        if m:
                            t = ts_ms(line)
                            self.servo_t.append(t)
                            self.servo.append((int(m.group(1)), int(m.group(2)), float(m.group(3)),
                                               int(m.group(4)), int(m.group(5))))
                            self.cum.append(self.cum[-1] + max(int(m.group(2)), 0))
                            self.last_t = t
                    elif "ctrlBoard_writeData(): euler_x=" in line:
                        m = RE_EULER.search(line)
                        if m:
                            self.euler_t.append(ts_ms(line))
                            self.euler.append(float(m.group(1)))
                    elif "istro::gps_writeData(): fix=1" in line:
                        m = RE_GPS.search(line)
                        if m:
                            self.gps_t.append(ts_ms(line))
                            self.gps.append((float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))))
                    elif "calibProcess2(): msg=\"calibration in progress" in line:
                        t = ts_ms(line)
                        if cal_last is None or t - cal_last > CALIB_GAP_S * 1000:
                            self.calib.append([t, t])
                        else:
                            self.calib[-1][1] = t
                        cal_last = t

    def pulses(self, t0, t1):
        """encoder pulses reported in frames with t0 < t <= t1"""
        i0 = bisect.bisect_right(self.servo_t, t0)
        i1 = bisect.bisect_right(self.servo_t, t1)
        return self.cum[i1] - self.cum[i0]

    def slice(self, times, values, t0, t1):
        return values[bisect.bisect_left(times, t0):bisect.bisect_right(times, t1)]

    def calib_overlap(self, t0, t1):
        cov = 0
        for a, b in self.calib:
            if b < t0 or a > t1:
                continue
            cov += min(b, t1) - max(a, t0)
        return cov / (t1 - t0) if t1 > t0 else 0.0


def measure(run, source, t0, t1):
    """metrics for the window [t0, t1]; None if there is nothing to measure"""
    dur = (t1 - t0) / 1000.0
    frames = run.slice(run.servo_t, run.servo, t0, t1)
    if dur <= 0 or not frames:
        return None
    pulses = run.pulses(t0, t1)
    velocities = [f[4] for f in frames]
    velocity = max(set(velocities), key=velocities.count)
    ircv500 = statistics.mean(f[2] for f in frames)
    fix_t = [t for t in run.gps_t if t0 <= t <= t1]
    fixes = run.slice(run.gps_t, run.gps, t0, t1)
    yaws = run.slice(run.euler_t, run.euler, t0, t1)
    seg = {
        "run": run.name, "source": source, "t_start": fmt_ts(t0), "t_end": fmt_ts(t1), "dur": dur,
        "velocity": velocity, "vel_offset": velocity - VEL_ZERO,
        "angle_mean": statistics.mean(f[3] for f in frames),
        "pulses": pulses, "pulse_rate": pulses / dur, "ircv500_mean": ircv500,
        "code_speed": ircv500 * CODE_SPEED_COEF,
        "gps_fixes": len(fixes), "gps_speed_mean": None, "gps_speed_median": None,
        "gps_disp": None, "gps_disp_dt": None, "gps_disp_speed": None, "pulses_disp": None,
        "m_per_pulse_speed": None, "m_per_pulse_disp": None,
        "course_mean": None, "course_spread": None,
        "yaw_mean": cmean(yaws) if yaws else None, "yaw_spread": cspread(yaws) if yaws else None,
        "calib_overlap": run.calib_overlap(t0, t1),
    }
    if fixes:
        speeds = [g[2] for g in fixes]
        seg["gps_speed_mean"] = statistics.mean(speeds)
        seg["gps_speed_median"] = statistics.median(speeds)
        moving = [g[3] for g in fixes if g[2] >= GPS_MOVING_SPEED]
        if moving:
            seg["course_mean"], seg["course_spread"] = cmean(moving), cspread(moving)
        if seg["pulse_rate"] > 0:
            seg["m_per_pulse_speed"] = seg["gps_speed_mean"] / seg["pulse_rate"]
    mpp = seg["m_per_pulse_speed"]
    if len(fixes) >= 2:
        ta, tb = fix_t[0], fix_t[-1]
        disp = haversine(fixes[0][0], fixes[0][1], fixes[-1][0], fixes[-1][1])
        seg["gps_disp"], seg["gps_disp_dt"] = disp, (tb - ta) / 1000.0
        if tb > ta:
            seg["gps_disp_speed"] = disp / seg["gps_disp_dt"]
        seg["pulses_disp"] = run.pulses(ta, tb)
        if seg["pulses_disp"] > 0:
            seg["m_per_pulse_disp"] = disp / seg["pulses_disp"]
            mpp = seg["m_per_pulse_disp"]
    seg["mode"] = "" if mpp is None else ("low" if mpp >= MODE_SPLIT_M_PER_PULSE else "high")
    return seg


def straight_segments(run, args):
    out = []
    n = len(run.servo)
    i = 0
    while i < n:
        st0, _, _, ang0, vel0 = run.servo[i]
        ok = st0 in (STATE_FWD, STATE_BCK) and vel0 != VEL_ZERO and abs(ang0 - SA_STRAIGHT) <= args.angle_tol
        if not ok:
            i += 1
            continue
        j = i
        while j + 1 < n:
            st, _, _, ang, vel = run.servo[j + 1]
            if vel != vel0 or st != st0 or abs(ang - SA_STRAIGHT) > args.angle_tol \
                    or run.servo_t[j + 1] - run.servo_t[j] > 500:
                break
            j += 1
        t0 = run.servo_t[i] + args.settle * 1000
        t1 = run.servo_t[j]
        i = j + 1
        if (t1 - t0) / 1000.0 < args.min_seg:
            continue
        seg = measure(run, "straight", t0, t1)
        if accept(seg, args):
            out.append(seg)
    return out


def accept(seg, args):
    """the straightness checks shared by both segment sources"""
    if not seg or seg["gps_fixes"] < 3:
        return False
    if seg["yaw_spread"] is not None and seg["yaw_spread"] > args.yaw_tol:
        return False
    if seg["course_spread"] is not None and seg["course_spread"] > COURSE_MAX_SPREAD:
        return False
    return True


def calib_segments(run, args):
    out, rejected = [], 0
    for a, b in run.calib:
        a += args.settle * 1000
        if (b - a) / 1000.0 < args.min_calib:
            continue
        seg = measure(run, "calib", a, b)
        if accept(seg, args):
            out.append(seg)
        else:
            rejected += 1
    return out, rejected


def segment_fields(s):
    return {
        "source": s["source"], "t_end": s["t_end"][11:], "dur": f2(s["dur"], 1),
        "velocity": s["velocity"], "vel_offset": s["vel_offset"], "angle_mean": f2(s["angle_mean"], 1),
        "pulses": s["pulses"], "pulse_rate": f2(s["pulse_rate"], 2), "ircv500_mean": f2(s["ircv500_mean"], 2),
        "code_speed": f2(s["code_speed"]), "gps_fixes": s["gps_fixes"],
        "gps_speed_mean": f2(s["gps_speed_mean"]), "gps_disp": f2(s["gps_disp"], 2),
        "gps_disp_speed": f2(s["gps_disp_speed"]), "pulses_disp": "" if s["pulses_disp"] is None else s["pulses_disp"],
        "m_per_pulse_speed": f2(s["m_per_pulse_speed"], 4), "m_per_pulse_disp": f2(s["m_per_pulse_disp"], 4),
        "course_spread": f2(s["course_spread"], 1), "yaw_spread": f2(s["yaw_spread"], 1),
        "calib_overlap": f2(s["calib_overlap"], 2), "mode": s["mode"],
    }


def med(vals):
    vals = [v for v in vals if v is not None]
    return statistics.median(vals) if vals else None


def summary_fields(segs):
    f = {"segments": len(segs), "total_s": f2(sum(s["dur"] for s in segs), 0),
         "mode_low": sum(1 for s in segs if s["mode"] == "low"), "mode_high": sum(1 for s in segs if s["mode"] == "high")}
    if not segs:
        return f
    mpp_disp = med(s["m_per_pulse_disp"] for s in segs)
    mpp_speed = med(s["m_per_pulse_speed"] for s in segs)
    # pooled: all GPS displacement over all pulses in between (long segments weigh more)
    disp_sum = sum(s["gps_disp"] for s in segs if s["gps_disp"] is not None and s["pulses_disp"])
    pulse_sum = sum(s["pulses_disp"] for s in segs if s["gps_disp"] is not None and s["pulses_disp"])
    f["m_per_pulse_disp_median"] = f2(mpp_disp, 4)
    f["m_per_pulse_disp_pooled"] = f2(disp_sum / pulse_sum if pulse_sum else None, 4)
    f["m_per_pulse_speed_median"] = f2(mpp_speed, 4)
    f["code_m_per_pulse"] = f"{CODE_M_PER_PULSE:.4f}"
    ratio = [s["gps_disp_speed"] / s["code_speed"] for s in segs if s["gps_disp_speed"] and s["code_speed"] > 0]
    f["gps_disp_speed_to_code_speed_median"] = f2(med(ratio), 2)
    pairs = [(s["pulse_rate"], s["gps_disp_speed"]) for s in segs if s["gps_disp_speed"] is not None]
    r = pearson([p[0] for p in pairs], [p[1] for p in pairs])
    f["corr_pulse_rate_gps_disp_speed"] = f2(r, 2)
    pairs = [(s["vel_offset"], s["gps_disp_speed"]) for s in segs if s["gps_disp_speed"] is not None]
    r = pearson([p[0] for p in pairs], [p[1] for p in pairs])
    f["corr_vel_offset_gps_disp_speed"] = f2(r, 2)
    r = pearson([s["vel_offset"] for s in segs], [s["pulse_rate"] for s in segs])
    f["corr_vel_offset_pulse_rate"] = f2(r, 2)
    return f


def emit_setpoints(t, run, segs):
    for v in sorted({s["velocity"] for s in segs}):
        x = [s for s in segs if s["velocity"] == v]
        emit("INFO", t, "setpoint", "per setpoint", run, {
            "source": x[0]["source"], "mode": x[0]["mode"] or "?", "velocity": v, "vel_offset": v - VEL_ZERO, "segments": len(x),
            "total_s": f2(sum(s["dur"] for s in x), 0),
            "pulse_rate_median": f2(med(s["pulse_rate"] for s in x), 2),
            "ircv500_median": f2(med(s["ircv500_mean"] for s in x), 2),
            "code_speed_median": f2(med(s["code_speed"] for s in x)),
            "gps_speed_median": f2(med(s["gps_speed_mean"] for s in x)),
            "gps_disp_speed_median": f2(med(s["gps_disp_speed"] for s in x)),
            "m_per_pulse_disp_median": f2(med(s["m_per_pulse_disp"] for s in x), 4),
        })


def fix_rows(run):
    """one row per GPS fix while the robot is commanded to move"""
    rows = []
    for t, g in zip(run.gps_t, run.gps):
        k = bisect.bisect_right(run.servo_t, t) - 1
        if k < 0:
            continue
        st, _, i500, ang, vel = run.servo[k]
        if vel == VEL_ZERO:
            continue
        yaws = run.slice(run.euler_t, run.euler, t - 1000, t)
        pulses_1s = run.pulses(t - 1000, t)
        mode = ""
        if g[2] >= GPS_MOVING_SPEED and pulses_1s > 0:
            mode = "low" if g[2] / pulses_1s >= MODE_SPLIT_M_PER_PULSE else "high"
        rows.append({"run": run.name, "t": fmt_ts(t), "gps_speed": g[2], "gps_course": g[3], "mode": mode,
                     "pulses_1s": pulses_1s, "ircv500": i500, "velocity": vel, "state": st,
                     "angle": ang, "yaw_spread_1s": f2(cspread(yaws), 1) if yaws else "",
                     "in_calib": 1 if run.calib_overlap(t - 1000, t) > 0.5 else 0})
    return rows


# CSV files use the Slovak locale so they open directly in a Slovak spreadsheet:
# ';' between columns and a decimal comma
CSV_DELIMITER = ";"
RE_DECIMAL = re.compile(r"^-?\d+\.\d+$")


def csv_value(v):
    if isinstance(v, float):
        return f"{v:.5f}".replace(".", ",")
    if isinstance(v, str) and RE_DECIMAL.match(v):
        return v.replace(".", ",")
    return v


def write_csv(path, rows):
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()), delimiter=CSV_DELIMITER)
        w.writeheader()
        for r in rows:
            w.writerow({k: csv_value(v) for k, v in r.items()})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="logout_* directory, or any directory above them")
    ap.add_argument("--csv", help="write one row per segment to this CSV file (e.g. odom_analyze_segments.csv)")
    ap.add_argument("--fixes-csv", help="write one row per GPS fix while moving to this CSV file (e.g. odom_analyze_fixes.csv)")
    ap.add_argument("-q", "--quiet", action="store_true", help="summaries only, no per-segment lines")
    ap.add_argument("--min-seg", type=float, default=5.0, help="minimum straight segment length in s (default 5)")
    ap.add_argument("--min-calib", type=float, default=4.0, help="minimum calibration window length in s after --settle (default 4)")
    ap.add_argument("--settle", type=float, default=1.5, help="s dropped at the start of a straight segment (default 1.5)")
    ap.add_argument("--angle-tol", type=int, default=20, help="steering tolerance around SA_STRAIGHT (default 20)")
    ap.add_argument("--yaw-tol", type=float, default=5.0, help="max IMU yaw spread in a straight segment (default 5)")
    args = ap.parse_args()

    all_segs, all_fixes, runs = [], [], 0
    for d in run_dirs(args.paths):
        run = Run(os.path.basename(os.path.normpath(d)))
        run.load(d)
        runs += 1
        calib, calib_rejected = calib_segments(run, args)
        segs = straight_segments(run, args) + calib
        segs.sort(key=lambda s: (s["source"], s["t_start"]))
        if not args.quiet:
            for s in segs:
                emit("INFO", ts_ms(s["t_start"]), "segment", "segment", run.name, segment_fields(s))
        for source in ("straight", "calib"):
            x = [s for s in segs if s["source"] == source]
            for mode in ("low", "high"):
                xm = [s for s in x if s["mode"] == mode]
                if xm:
                    emit_setpoints(run.last_t, run.name, xm)
            f = {"source": source, **summary_fields(x)}
            if source == "calib":
                f["calib_windows_rejected"] = calib_rejected
            if not run.servo:
                f["note"] = "no control-board telemetry (DEBUG ctrlBoard_writeData lines)"
            elif not run.gps:
                f["note"] = "no GPS fix in this run"
            emit("INFO", run.last_t, "summary", "run summary", run.name, f)
        all_segs += segs
        if args.fixes_csv:
            all_fixes += fix_rows(run)

    if runs > 1:
        for source in ("straight", "calib"):
            x = [s for s in all_segs if s["source"] == source]
            for mode in ("low", "high"):
                xm = [s for s in x if s["mode"] == mode]
                if xm:
                    emit_setpoints(now_ms(), f"{runs} runs", xm)
                    emit("INFO", now_ms(), "summary", "total summary per mode", f"{runs} runs",
                         {"source": source, "mode": mode, **summary_fields(xm)})
            emit("INFO", now_ms(), "summary", "total summary", f"{runs} runs", {"source": source, **summary_fields(x)})

    if args.csv and all_segs:
        write_csv(args.csv, all_segs)
    if args.fixes_csv and all_fixes:
        write_csv(args.fixes_csv, all_fixes)


if __name__ == "__main__":
    main()
