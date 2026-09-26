#!/bin/bash
# Stall watchdog: one fixed-width line per second with every indicator side by
# side, marked STALL on the seconds where message delivery actually hiccuped,
# so correlations can be read straight off the column rather than
# reconstructed afterwards.
#
#   ./script/test/test_stall_watch.sh                     # default topic
#   ./script/test/test_stall_watch.sh /robot/camera_rear_data
#
# WHY: the delivery stalls chased on 2026-09-07 were episodic -- minutes of
# ~2 s gaps, then minutes of nothing. Every manual measurement arrived after
# the episode had passed, which is how CPU, swap, memory reclaim and disk I/O
# each got blamed in turn and then cleared. Sampling everything continuously
# and flagging the stall seconds makes "does this move when delivery breaks?"
# a matter of reading down a column.
#
# Columns (all rates are per second, deltas since the previous line):
#   load  1-minute load average -- above the core count means the run queue is
#         longer than the machine is wide, which is the state this box sits in
#   runq  runnable/total threads, straight from /proc/loadavg
#   cpu   busy percentage across all cores
#   free  MemAvailable in MB
#   swpi  pages read back IN from swap  -- each one blocks the faulting process
#   swpo  pages written OUT to swap
#   mflt  major page faults -- a disk read on the critical path
#   astl  allocation stalls: direct reclaim, a process blocked finding memory
#   dio   disk I/Os in progress on the device backing out/
#   gap   worst inter-arrival gap the probe reported for this second (ms)
#
# Composes the tools that already work rather than replacing them:
# test_camera_delay does the timing in C++ (no interpreter jitter, no timer of
# its own), this samples /proc alongside it.
#
# Fast DDS's own statistics module would have been the obvious place to look
# and IS compiled into this Jazzy install (17 _fastdds_statistics_* topics
# including history2history_latency), but it is not reachable from here: those
# topics carry eProsima IDL types rather than ROS messages, so ros2 topic
# never sees them. Reading them needs a native Fast DDS subscriber plus
# FASTDDS_STATISTICS set on every node being watched, and all of them
# restarted. Verified empirically, not assumed.
set -e

cd "$(dirname "$0")/../.."
TOPIC="${1:-/robot/camera_front_data}"
PROBE_OUT=$(mktemp /tmp/stall_watch_probe.XXXXXX)

# Device backing out/, resolved the same way save_node's
# detectSaveDeviceName() does: st_dev -> /sys/dev/block/<major>:<minor>.
DEV=$(basename "$(readlink -f "/sys/dev/block/$(stat -c '%d' out 2>/dev/null | \
      awk '{printf "%d:%d", int($1/256), $1%256}')" 2>/dev/null)" 2>/dev/null || true)
[ -z "$DEV" ] && DEV="nvme0n1p1"

cleanup() { kill "$PROBE_PID" 2>/dev/null || true; rm -f "$PROBE_OUT"; }
trap cleanup EXIT INT TERM

stdbuf -oL ./script/test/test_camera_delay.sh "$TOPIC" 5 > "$PROBE_OUT" 2>&1 &
PROBE_PID=$!

vm() { awk -v k="$1" '$1==k{print $2+0; f=1} END{if(!f)print 0}' /proc/vmstat; }

pa=$(( $(vm allocstall_normal) + $(vm allocstall_movable) ))
pf=$(vm pgmajfault); pi=$(vm pswpin); po=$(vm pswpout)
read -r _ u n s idle rest < /proc/stat; pb=$((u+n+s)); pt=$((u+n+s+idle))
plines=0

echo "watching $TOPIC (disk $DEV) -- Ctrl+C to stop"
printf "%-8s %5s %6s %4s %6s %5s %5s %5s %5s %4s %7s %s\n" \
       "time" "load" "runq" "cpu" "free" "swpi" "swpo" "mflt" "astl" "dio" "gap" ""

while kill -0 "$PROBE_PID" 2>/dev/null; do
    sleep 1

    a=$(( $(vm allocstall_normal) + $(vm allocstall_movable) ))
    f=$(vm pgmajfault); i=$(vm pswpin); o=$(vm pswpout)
    read -r _ u n s idle rest < /proc/stat; b=$((u+n+s)); t=$((u+n+s+idle))

    cpu=0
    [ $((t - pt)) -gt 0 ] && cpu=$(( (b - pb) * 100 / (t - pt) ))

    # Only the probe lines added since the previous second, so "gap" and the
    # STALL flag describe THIS second and not the whole run so far.
    lines=$(wc -l < "$PROBE_OUT")
    gap=$(tail -n +$((plines + 1)) "$PROBE_OUT" | head -n $((lines - plines)) \
          | awk '$4 ~ /^[0-9.]+$/ {if ($4+0 > m) m = $4+0} END {printf "%.0f", m+0}')
    stall=$(tail -n +$((plines + 1)) "$PROBE_OUT" | head -n $((lines - plines)) | grep -c STALL || true)
    plines=$lines

    printf "%-8s %5s %6s %3d%% %6s %5d %5d %5d %5d %4s %7s %s\n" \
        "$(date +%H:%M:%S)" \
        "$(cut -d' ' -f1 /proc/loadavg)" \
        "$(cut -d' ' -f4 /proc/loadavg)" \
        "$cpu" \
        "$(awk '/^MemAvailable/{printf "%d",$2/1024}' /proc/meminfo)" \
        "$((i - pi))" "$((o - po))" "$((f - pf))" "$((a - pa))" \
        "$(awk -v d="$DEV" '$3==d{print $12; f=1} END{if(!f)print "-"}' /proc/diskstats)" \
        "$gap" \
        "$([ "$stall" -gt 0 ] && echo '<-- STALL' || echo '')"

    pa=$a; pf=$f; pi=$i; po=$o; pb=$b; pt=$t
done
