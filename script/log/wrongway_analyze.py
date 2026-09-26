#!/usr/bin/env python3
"""Find WRONGWAY reversals in istro_<EVENT_TAG> logs (istro_rt2025.log, istro_rt2026.log, ...) and say what happened to each one.

Usage:
    wrongway_analyze.py LOGDIR [LOGDIR ...]      # logout_* dirs, or any dir above them
    wrongway_analyze.py -f LOGDIR                # failures only
    wrongway_analyze.py -v LOGDIR                # + board timeline for each failure

Output is in the same line format as istro_<EVENT_TAG>.log, timestamped when the judged state
started, so it can be grepped the same way. Needs DEBUG logging of ctrlBoard_writeData() and
ControlBoard::write() -- without it every event reports reason="no_telemetry".

Each wrongway cycle is judged on its BACK4 phase (3500-6000 ms after "wrongway start!",
planner_node.cpp WRONGWAY_TIME3_STOP/TIME4_BACK). BACK2 is only 250 ms, too short to judge.

  WARN  wrongway backward FAILED  reason=no_wheel_motion  board accepted reverse (ctrlb_velocity<335)
                                                          for >= 1 s, encoders reported nothing
  WARN  wrongway backward FAILED  reason=board_ignored    reverse sent for >= 1 s, board never took it
  INFO  wrongway backward OK
  INFO  wrongway backward skipped reason=emergency|manual|process_stop|board_obstacle|short|
                                         interrupted|no_telemetry  -- robot was not supposed to move
                                  reason=cancelled_emergency|cancelled_loadarea  -- planner_node's
                                         wrongwayHold() cancelled the cycle before BACK4 ended

Details and the reasoning behind the thresholds: doc/ai/01_architecture.md, WRONG_WAY "Backward check".
"""

import argparse
import datetime
import gzip
import os
import re
import sys

# istro_<tag>.log, rotated istro_<tag>.HH_MM.log[.gz] -- any EVENT_TAG, so old archives still work
LOG_NAME = re.compile(r"^istro_[a-z]+\d+(\.\d\d_\d\d)?\.log(\.gz)?$")

VEL_ZERO = 335
CTRLB_STATE_EBTN = 8
CTRLB_STATE_OBST = (6, 7)           # OBSTB, OBSTA -- board refuses to reverse
BACK4_FROM, BACK4_TO = 3500, 6000   # ms after "wrongway start!"
JUDGE_MIN_MS = 1000                 # shorter reversals are not judged
IRCV500_MOVING = 5.0                # = planner's WRONGWAY_IRCV500_MIN_VALUE
IRCV500_TAIL_MS = 700               # ircv500 is a 500 ms window, let it catch up after the phase

STATE_NAMES = {0: "START", 1: "STOP", 2: "FWD", 3: "BCK", 4: "RECO", 5: "OBSTF", 6: "OBSTB", 7: "OBSTA", 8: "EBTN"}

RE_SERVO = re.compile(r'state=(\d+), ctrlb_ircv=(-?\d+), ircv500=([\d.]+), ctrlb_angle=(\d+), ctrlb_velocity=(-?\d+)')
RE_S2 = re.compile(r'data="S2(\d+)"')
RE_DISP = re.compile(r'data="D([AMP])')
RE_PSTOP = re.compile(r'process_stop=(-?\d+)')
RE_CANCEL = re.compile(r'msg="wrongway cancelled!", reason="([^"]*)"')
RE_CHECK = re.compile(r'wrongwayBackCheck\(\): msg="([^"]*)"(?:, reason="([^"]*)")?')

_day_cache = {}


def ts_ms(line):
    """'2026-09-12 11:29:36,755 ...' -> ms since epoch-ish (only differences matter)."""
    d = line[:10]
    base = _day_cache.get(d)
    if base is None:
        base = datetime.date.fromisoformat(d).toordinal() * 86400000
        _day_cache[d] = base
    return base + int(line[11:13]) * 3600000 + int(line[14:16]) * 60000 + int(line[17:19]) * 1000 + int(line[20:23])


def fmt_ts(ms):
    day, rest = divmod(ms, 86400000)
    date = datetime.date.fromordinal(day).isoformat()
    h, rest = divmod(rest, 3600000)
    m, rest = divmod(rest, 60000)
    s, milli = divmod(rest, 1000)
    return f"{date} {h:02d}:{m:02d}:{s:02d},{milli:03d}"


def log_files(run_dir):
    files = [os.path.join(run_dir, f) for f in os.listdir(run_dir)
             if LOG_NAME.match(f)]

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
            print(f"wrongway_analyze.py: no istro_<tag> logs under {p}", file=sys.stderr)


class Event:
    def __init__(self, t0, trigger):
        self.t0 = t0
        self.trigger = trigger
        self.end = "timeout"
        self.frames = []            # (t, state, ircv, ircv500, angle, velocity)
        self.back_writes = []       # t
        self.zero_writes = 0
        self.modes = {}
        self.pstop = [0, 0]         # stop=1 count, total
        self.checks = []            # drive_node wrongwayBackCheck lines (msg, reason)
        self.mode_before = "?"
        self.cancel = None          # (t, reason) from planner_node's "wrongway cancelled!"


def judge(ev):
    """-> (level, msg, reason, t_state, fields)"""
    win = [f for f in ev.frames if BACK4_FROM <= f[0] - ev.t0 <= BACK4_TO]
    t_win = ev.t0 + BACK4_FROM
    f = {"trigger": ev.trigger, "wrongway_start": fmt_ts(ev.t0)[11:], "wrongway_end": ev.end}

    back = [t for t in ev.back_writes if BACK4_FROM <= t - ev.t0 <= BACK4_TO]
    f["back_writes"] = len(back)
    f["zero_writes"] = ev.zero_writes
    mode = max(ev.modes, key=ev.modes.get) if ev.modes else ev.mode_before
    f["mode"] = {"A": "AUTONOMOUS", "M": "MANUAL", "P": "PROGRAM"}.get(mode, "?")
    f["process_stop"] = f"{ev.pstop[0]}/{ev.pstop[1]}"
    if ev.checks:
        f["drive_check"] = ",".join((r or m) for m, r in ev.checks)

    if ev.cancel is not None and ev.cancel[0] - ev.t0 < BACK4_TO:
        f["cancel_dt"] = ev.cancel[0] - ev.t0
        return "INFO", "wrongway backward skipped", "cancelled_" + ev.cancel[1], ev.cancel[0], f
    if ev.end in ("finished", "reset", "next_start") and not win and not back:
        return "INFO", "wrongway backward skipped", "interrupted", t_win, f
    if not win:
        return "INFO", "wrongway backward skipped", "no_telemetry", t_win, f

    states = {}
    for fr in win:
        states[fr[1]] = states.get(fr[1], 0) + 1
    f["ctrlb_states"] = ",".join(f"{STATE_NAMES.get(s, s)}x{n}" for s, n in sorted(states.items()))
    f["frames"] = len(win)
    f["angle_min"] = min(fr[4] for fr in win)
    f["angle_max"] = max(fr[4] for fr in win)

    # board-accepted reverse: contiguous frames with ctrlb_velocity < VEL_ZERO
    acc_dt, acc_first, acc_last, prev, pulses, ircv500_max = 0, None, None, None, 0, 0.0
    for fr in win:
        if 0 <= fr[5] < VEL_ZERO:
            if prev is not None:
                acc_dt += fr[0] - prev
            prev = fr[0]
            acc_first = fr[0] if acc_first is None else acc_first
            acc_last = fr[0]
            pulses += max(fr[2], 0)
        else:
            prev = None
    if acc_last is not None:
        for fr in ev.frames:
            if acc_first <= fr[0] <= acc_last + IRCV500_TAIL_MS:
                ircv500_max = max(ircv500_max, fr[3])
    vels = sorted({fr[5] for fr in win})
    f["ctrlb_velocity"] = ",".join(str(v) for v in vels)
    f["board_accepted_dt"] = acc_dt
    f["ircv_pulses"] = pulses
    f["ircv500_max"] = f"{ircv500_max:.2f}"
    if back and acc_first is not None:
        f["accept_delay"] = acc_first - back[0]

    n = len(win)
    if states.get(CTRLB_STATE_EBTN, 0) >= n / 2:
        return "INFO", "wrongway backward skipped", "emergency", t_win, f
    if mode == "M":
        return "INFO", "wrongway backward skipped", "manual", t_win, f
    if sum(states.get(s, 0) for s in CTRLB_STATE_OBST) >= n / 2 and acc_first is None:
        return "INFO", "wrongway backward skipped", "board_obstacle", t_win, f

    if acc_first is not None:
        f["back_dt"] = acc_dt
        if ircv500_max >= IRCV500_MOVING:
            return "INFO", "wrongway backward OK", None, acc_first, f
        if acc_dt >= JUDGE_MIN_MS:
            return "WARN", "wrongway backward FAILED", "no_wheel_motion", acc_first, f
        return "INFO", "wrongway backward skipped", "short", acc_first, f

    if back:
        f["back_dt"] = back[-1] - back[0]
        if f["back_dt"] >= JUDGE_MIN_MS:
            return "WARN", "wrongway backward FAILED", "board_ignored", back[0], f
        return "INFO", "wrongway backward skipped", "short", back[0], f

    if ev.pstop[1] and ev.pstop[0] >= ev.pstop[1] / 2:
        return "INFO", "wrongway backward skipped", "process_stop", t_win, f
    return "INFO", "wrongway backward skipped", "no_back_command", t_win, f


def emit(level, t, method, msg, reason, run, fields):
    parts = [f'run="{run}"', f'msg="{msg}"']
    if reason:
        parts.append(f'reason="{reason}"')
    for k, v in fields.items():
        parts.append(f'{k}="{v}"' if isinstance(v, str) else f"{k}={v}")
    print(f"{fmt_ts(t)} {level:<5} [analyze] wwAnalyze::{method}(): " + ", ".join(parts))


def timeline(ev, run):
    last = None
    for fr in ev.frames:
        dt = fr[0] - ev.t0
        if dt < BACK4_FROM - 500 or dt > BACK4_TO + IRCV500_TAIL_MS:
            continue
        key = (fr[1], fr[5], fr[2] > 0, int(fr[3] // IRCV500_MOVING))
        if key != last:
            print(f"{fmt_ts(fr[0])} DEBUG [analyze] wwAnalyze::timeline(): run=\"{run}\", phase_dt={dt}, "
                  f"ctrlb_state={fr[1]}, ctrlb_velocity={fr[5]}, ctrlb_ircv={fr[2]}, ircv500={fr[3]:.2f}, ctrlb_angle={fr[4]}")
            last = key


def analyze_run(run_dir, args, totals):
    run = os.path.basename(os.path.normpath(run_dir))
    counts = {}
    ev = None
    trigger = "?"
    last_mode = "?"
    last_t = None

    def close(e):
        level, msg, reason, t, fields = judge(e)
        key = msg.split()[-1] + ("_" + reason if reason else "")
        counts[key] = counts.get(key, 0) + 1
        if level == "WARN" or not args.failures:
            emit(level, t, "event", msg, reason, run, fields)
            if level == "WARN" and args.verbose:
                timeline(e, run)

    for path in log_files(run_dir):
        opener = gzip.open if path.endswith(".gz") else open
        with opener(path, "rt", errors="replace") as fh:
            for line in fh:
                if len(line) < 24 or line[4] != "-" or line[13] != ":":
                    continue
                if "wrongway" in line:
                    t = ts_ms(line)
                    if 'msg="wrongway start!"' in line:
                        if ev:
                            ev.end = "next_start"
                            close(ev)
                        ev = Event(t, trigger)
                        ev.mode_before = last_mode
                        trigger = "?"
                    elif 'msg="wrongway forced!"' in line:
                        trigger = "force"
                    elif 'msg="wrongway detected!"' in line:
                        trigger = "check"
                    elif ev and 'msg="wrongway finished!"' in line:
                        ev.end = "finished"
                    elif ev and "wrongway-check interrupted" in line and "wrongwayReset" in line:
                        ev.end = "reset"
                    elif ev and 'msg="wrongway cancelled!"' in line:
                        m = RE_CANCEL.search(line)
                        ev.cancel = (t, m.group(1) if m else "?")
                        ev.end = "cancelled"
                    elif ev and "wrongwayBackCheck" in line:
                        m = RE_CHECK.search(line)
                        if m:
                            ev.checks.append((m.group(1), m.group(2)))
                    continue
                servo = "ctrlBoard_writeData(): state=" in line
                write = (not servo) and "ControlBoard::write(): data=" in line
                planner = (not servo and not write) and 'process_angle("WRONG_WAY' in line
                if not (servo or write or planner):
                    continue
                t = ts_ms(line)
                last_t = t
                if ev is not None and t - ev.t0 > BACK4_TO + IRCV500_TAIL_MS:
                    close(ev)
                    ev = None
                if ev is None:
                    if write:
                        m = RE_DISP.search(line)
                        if m:
                            last_mode = m.group(1)
                    continue
                if servo:
                    m = RE_SERVO.search(line)
                    if m:
                        ev.frames.append((t, int(m.group(1)), int(m.group(2)), float(m.group(3)),
                                          int(m.group(4)), int(m.group(5))))
                elif write:
                    dt = t - ev.t0
                    m = RE_S2.search(line)
                    if m:
                        v = int(m.group(1))
                        if v < VEL_ZERO:
                            ev.back_writes.append(t)
                        elif BACK4_FROM <= dt <= BACK4_TO:
                            ev.zero_writes += 1
                        continue
                    m = RE_DISP.search(line)
                    if m:
                        last_mode = m.group(1)
                        if BACK4_FROM <= dt <= BACK4_TO:
                            ev.modes[m.group(1)] = ev.modes.get(m.group(1), 0) + 1
                else:
                    if BACK4_FROM <= t - ev.t0 <= BACK4_TO:
                        m = RE_PSTOP.search(line)
                        if m:
                            ev.pstop[1] += 1
                            ev.pstop[0] += int(m.group(1)) > 0
    if ev:
        close(ev)

    if not counts and not args.failures:
        return
    summary = {k: counts[k] for k in sorted(counts)}
    for k, v in counts.items():
        totals[k] = totals.get(k, 0) + v
    emit("INFO", last_t if last_t is not None else 0, "summary", "run summary", None, run,
         {"wrongway": sum(counts.values()), **summary})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="logout_* directory, or any directory above them")
    ap.add_argument("-f", "--failures", action="store_true", help="print only failed reversals")
    ap.add_argument("-v", "--verbose", action="store_true", help="board timeline (DEBUG lines) for each failure")
    args = ap.parse_args()
    totals = {}
    runs = list(run_dirs(args.paths))
    for d in runs:
        analyze_run(d, args, totals)
    if len(runs) > 1 and totals:
        n = datetime.datetime.now()
        now_ms = n.date().toordinal() * 86400000 + ((n.hour * 60 + n.minute) * 60 + n.second) * 1000 + n.microsecond // 1000
        emit("INFO", now_ms, "summary", "total summary", None, f"{len(runs)} runs",
             {"wrongway": sum(totals.values()), **{k: totals[k] for k in sorted(totals)}})


if __name__ == "__main__":
    main()
