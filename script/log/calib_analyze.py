#!/usr/bin/env python3
"""Check the compass calibration in istro_<EVENT_TAG> logs: GPS course vs IMU (BNO055) yaw.

Usage:
    calib_analyze.py LOGDIR [LOGDIR ...]      # logout_* dirs, or any dir above them
    calib_analyze.py -v LOGDIR                # + one DEBUG line per straight segment
    calib_analyze.py --min-seg 7 LOGDIR       # segment length, default 4 s

Output is in the same line format as istro_<EVENT_TAG>.log, so it greps the same way.

Two calibrations run in planner_node (planner_node.cpp calibProcess()):
  calib_  (calibProcess1) drives navigation: yawc = yaw - calib.yaw + calib.azimuth. Started with
          -cg <gps_azimuth> -ca <imu_yaw> it is valid from the start and must NOT change during a run
          (it would only re-calibrate after 12000000 ms). Any calibProcess1 activity is a WARN then.
  calib2_ (calibProcess2, ISTRO_CALIB2) re-calibrates every 2 min and is logged only, never used.
          Its "calibration finished!" lines and its per-tick "calibration in progress..." samples
          (TRACE) are what this script measures the configured constants against.

delta = gps_course - imu_yaw, normalised to (-180, 180]. -cg 300 -ca 0 means delta -60.

  INFO  calibration config         configured delta per run
  WARN  navigation calibration changed/active   calibProcess1 finished/needed while configured
  INFO  calibration finished       one calib2_ result; WARN if it is > EVENT_DIFF_WARN from configured
  DEBUG straight segment (-v)      a >= --min-seg s stretch passing calibProcess()'s own checks
  INFO  run summary / total summary    counts, segment median/mean/stdev, interruption reasons;
                                   WARN if the segment median is > MEDIAN_DIFF_WARN from configured

Details and the reasoning behind the checks: doc/260912_test_sadJK/README.md section 12.
"""

import argparse
import datetime
import gzip
import math
import os
import re
import statistics
import sys

# istro_<tag>.log, rotated istro_<tag>.HH_MM.log[.gz] -- any EVENT_TAG
LOG_NAME = re.compile(r"^istro_[a-z]+\d+(\.\d\d_\d\d)?\.log(\.gz)?$")

ANGLE_OK = 999998
# calibProcess() checks (planner_node.cpp CALIB_*), applied to straight segments
SPEED_CHK_S, ANGLE_CHK_S = 2.5, 3.5
GPS_SPEED_MIN, IRCV500_MIN = 0.2, 5.0
COURSE_MAX_DIFF, YAW_MAX_DIFF = 15.0, 5.0
SEG_GAP_S = 0.3                # a gap between samples longer than this ends a segment
EVENT_DIFF_WARN = 20.0         # one calib2_ result this far from the configured delta
MEDIAN_DIFF_WARN = 10.0        # segment median this far from the configured delta
MIN_SEGMENTS = 5               # fewer segments -> no verdict

RE_SAMPLE = re.compile(r'gps_course=([-\d.]+), yaw=([-\d.]+), gps_speed=([-\d.]+), .*ircv500=([-\d.]+)')
RE_FINISHED = re.compile(r'calib\.yaw=([-\d.]+), calib\.azimuth=([-\d.]+)')
RE_REASON = re.compile(r'reason="([^"]*)"')

_day_cache = {}


def ts_ms(line):
    d = line[:10]
    base = _day_cache.get(d)
    if base is None:
        base = datetime.date.fromisoformat(d).toordinal() * 86400000
        _day_cache[d] = base
    return base + int(line[11:13]) * 3600000 + int(line[14:16]) * 60000 + int(line[17:19]) * 1000 + int(line[20:23])


def fmt_ts(ms):
    day, rest = divmod(ms, 86400000)
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
    m = cmean(vals)
    d = [norm(v - m) for v in vals]
    return max(d) - min(d)


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
            print(f"calib_analyze.py: no istro_<tag> logs under {p}", file=sys.stderr)


def emit(level, t, method, msg, run, fields):
    parts = [f'run="{run}"', f'msg="{msg}"']
    for k, v in fields.items():
        parts.append(f'{k}="{v}"' if isinstance(v, str) else f"{k}={v}")
    print(f"{fmt_ts(t)} {level:<5} [analyze] calibAnalyze::{method}(): " + ", ".join(parts))


def configured_delta(cg, ca):
    if cg is None or ca is None or cg >= ANGLE_OK or ca >= ANGLE_OK:
        return None
    return norm(cg - ca)


def judge_segment(samples, min_seg):
    """samples: [(t_ms, course, yaw, speed, ircv500)] -> segment dict, or None if it fails the checks."""
    if len(samples) < 6:
        return None
    t0 = samples[0][0]
    dur = (samples[-1][0] - t0) / 1000.0
    if dur < min_seg:
        return None
    late = [x for x in samples if (x[0] - t0) >= SPEED_CHK_S * 1000]
    if not all(x[3] >= GPS_SPEED_MIN and x[4] >= IRCV500_MIN for x in late):
        return None
    ang = [x for x in samples if (x[0] - t0) >= ANGLE_CHK_S * 1000] or samples
    cs, ys = cspread([x[1] for x in ang]), cspread([x[2] for x in ang])
    if cs > COURSE_MAX_DIFF or ys > YAW_MAX_DIFF:
        return None
    course, yaw = cmean([x[1] for x in samples]), cmean([x[2] for x in samples])
    return {"t": t0, "dur": dur, "course": course, "yaw": yaw, "delta": norm(course - yaw),
            "course_spread": cs, "yaw_spread": ys, "n": len(samples)}


def stats(deltas):
    return {"median": f"{statistics.median(deltas):.1f}", "mean": f"{statistics.mean(deltas):.1f}",
            "stdev": f"{statistics.pstdev(deltas):.1f}", "min": f"{min(deltas):.1f}", "max": f"{max(deltas):.1f}"}


def verdict(deltas, conf):
    if conf is None:
        return "INFO", "no configured constants (-cg/-ca) to compare"
    if len(deltas) < MIN_SEGMENTS:
        return "INFO", f"not enough straight segments for a verdict (< {MIN_SEGMENTS})"
    diff = norm(statistics.median(deltas) - conf)
    if abs(diff) > MEDIAN_DIFF_WARN:
        return "WARN", f"configured constants disagree with the drive (median off by {diff:.1f} deg)"
    return "INFO", f"configured constants consistent with the drive (median off by {diff:.1f} deg)"


def analyze_run(run_dir, args, total):
    run = os.path.basename(os.path.normpath(run_dir))
    cg = ca = None
    conf_logged = None
    last_t = 0
    finished, calib1, reasons, segs = [], [], {}, []
    cur, last_sample = [], None

    def close():
        nonlocal cur
        seg = judge_segment(cur, args.min_seg)
        if seg:
            segs.append(seg)
            if args.verbose:
                emit("DEBUG", seg["t"], "segment", "straight segment", run,
                     {"dur": f"{seg['dur']:.1f}", "gps_course": f"{seg['course']:.1f}", "imu_yaw": f"{seg['yaw']:.1f}",
                      "delta": f"{seg['delta']:.1f}", "course_spread": f"{seg['course_spread']:.1f}",
                      "yaw_spread": f"{seg['yaw_spread']:.1f}", "samples": seg["n"]})
        cur = []

    for path in log_files(run_dir):
        opener = gzip.open if path.endswith(".gz") else open
        with opener(path, "rt", errors="replace") as fh:
            for line in fh:
                if len(line) < 24 or line[4] != "-" or line[13] != ":":
                    continue
                if "printArguments" in line:
                    if "CalibGpsAzimuth=" in line:
                        cg = int(line.rsplit("=", 1)[1])
                    elif "CalibImuYaw=" in line:
                        ca = int(line.rsplit("=", 1)[1])
                        conf = configured_delta(cg, ca)
                        key = (cg, ca)
                        if key != conf_logged:
                            t = ts_ms(line)
                            emit("INFO" if conf is not None else "WARN", t, "config", "calibration config", run,
                                 {"calib_gps_azimuth": cg, "calib_imu_yaw": ca,
                                  "configured_delta": f"{conf:.1f}" if conf is not None else "none"})
                            conf_logged = key
                    continue
                if "calibProcess" not in line and "calibReset" not in line:
                    continue
                t = ts_ms(line)
                last_t = t
                conf = configured_delta(cg, ca)
                idx = 1 if ("calibProcess1" in line or "calibReset1" in line) else 2
                if 'msg="calibration in progress' in line:
                    if idx != 2:
                        continue
                    m = RE_SAMPLE.search(line)
                    if not m:
                        continue
                    if last_sample is not None and t - last_sample > SEG_GAP_S * 1000:
                        close()
                    cur.append((t, float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))))
                    last_sample = t
                    continue
                if idx == 2:
                    close()
                    last_sample = None
                if 'msg="calibration interrupted' in line:
                    m = RE_REASON.search(line)
                    key = f"calib{idx}_{m.group(1) if m else '?'}"
                    reasons[key] = reasons.get(key, 0) + 1
                    continue
                if idx == 1 and ('msg="calibration finished' in line or 'msg="calibration needed' in line
                                 or 'msg="calibration start' in line):
                    what = "finished" if "finished" in line else ("needed" if "needed" in line else "start")
                    calib1.append(what)
                    if conf is not None:
                        emit("WARN", t, "event", f"navigation calibration {what} although -cg/-ca are configured", run,
                             {"line": line[line.index("msg="):].strip().replace('"', "'")})
                    elif what == "finished":
                        emit("INFO", t, "event", "navigation calibration finished (no -cg/-ca, values are used)", run,
                             {"line": line[line.index("msg="):].strip().replace('"', "'")})
                    continue
                if idx == 2 and 'msg="calibration finished' in line:
                    m = RE_FINISHED.search(line)
                    if not m:
                        continue
                    yaw, course = float(m.group(1)), float(m.group(2))
                    delta = norm(course - yaw)
                    fields = {"imu_yaw": f"{yaw % 360:.1f}", "gps_course": f"{course % 360:.1f}", "delta": f"{delta:.1f}"}
                    level = "INFO"
                    if conf is not None:
                        diff = norm(delta - conf)
                        fields["diff_from_configured"] = f"{diff:.1f}"
                        level = "WARN" if abs(diff) > EVENT_DIFF_WARN else "INFO"
                    finished.append(delta)
                    emit(level, t, "event", "calibration finished", run, fields)
    close()

    conf = configured_delta(cg, ca)
    deltas = [s["delta"] for s in segs]
    fields = {"calib2_finished": len(finished), "calib1_events": len(calib1),
              f"segments_{args.min_seg:g}s": len(segs), "segments_total_s": f"{sum(s['dur'] for s in segs):.0f}"}
    if finished:
        fields["finished_median"] = f"{statistics.median(finished):.1f}"
    if deltas:
        fields.update({f"segment_{k}": v for k, v in stats(deltas).items()})
    if conf is not None:
        fields["configured_delta"] = f"{conf:.1f}"
    fields["interrupted"] = ",".join(f"{k}x{v}" for k, v in sorted(reasons.items(), key=lambda kv: -kv[1])) or "none"
    level, text = verdict(deltas, conf)
    fields["verdict"] = text
    if not segs and not finished:
        fields["note"] = "no calibration samples -- TRACE logging of calibProcess2 off, or no straight driving"
    emit(level, last_t, "summary", "run summary", run, fields)

    total["runs"] += 1
    total["finished"] += finished
    total["segs"] += segs
    total["calib1"] += len(calib1)
    total["confs"].add(conf)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="logout_* directory, or any directory above them")
    ap.add_argument("-v", "--verbose", action="store_true", help="one DEBUG line per straight segment")
    ap.add_argument("--min-seg", type=float, default=4.0, help="minimum straight segment length in s (default 4)")
    args = ap.parse_args()
    total = {"runs": 0, "finished": [], "segs": [], "calib1": 0, "confs": set()}
    for d in run_dirs(args.paths):
        analyze_run(d, args, total)
    if total["runs"] > 1:
        deltas = [s["delta"] for s in total["segs"]]
        confs = [c for c in total["confs"] if c is not None]
        conf = confs[0] if len(confs) == 1 else None
        fields = {"calib2_finished": len(total["finished"]), "calib1_events": total["calib1"],
                  f"segments_{args.min_seg:g}s": len(deltas)}
        if total["finished"]:
            fields["finished_median"] = f"{statistics.median(total['finished']):.1f}"
        if deltas:
            fields.update({f"segment_{k}": v for k, v in stats(deltas).items()})
            for lo in range(0, 360, 45):
                x = [s["delta"] for s in total["segs"] if lo <= s["course"] < lo + 45]
                if x:
                    fields[f"course_{lo:03d}_{lo + 45:03d}"] = f"n{len(x)}:{statistics.median(x):.1f}"
        if conf is not None:
            fields["configured_delta"] = f"{conf:.1f}"
        elif len(confs) > 1:
            fields["configured_delta"] = "differs between runs"
        level, text = verdict(deltas, conf)
        fields["verdict"] = text
        emit(level, now_ms(), "summary", "total summary", f"{total['runs']} runs", fields)


if __name__ == "__main__":
    main()
