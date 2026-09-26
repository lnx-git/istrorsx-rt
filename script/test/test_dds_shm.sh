#!/bin/bash
# Shows how much shared memory Fast DDS is actually holding, per node and in
# total, and whether the large-message profile is in effect at all.
#
#   ./script/test/test_dds_shm.sh
#
# WHY: CameraFrame is 1735680 B. Fast DDS's default SHM segment is 512 KB
# (549408 B with overhead), so every frame is pushed through a window a third
# its size, and under RELIABLE QoS the writer blocks waiting for the reader to
# drain -- the ~2 s publisher stalls on /robot/camera_front_data. The fix is
# conf/fastdds_large_msg.xml, wired in by _setup_env.sh. This says whether a
# running system picked it up, since a node started from a shell that did not
# have the variable set keeps the 512 KB default silently.
#
# The segment is committed RAM, not lazily faulted in (measured: a 17152 KB
# segment reports 17156 KB under du), and every participant creates its own --
# so the total here is real memory, worth watching on the Orin Nano's 8 GB
# where the GPU and the TensorRT model share the same pool.
set -e

cd "$(dirname "$0")/../.."

COLOR_W=$((640 * 480 * 3))
DEPTH_W=$((848 * 480 * 2))
MSG=$((COLOR_W + DEPTH_W))

# The expected size comes from the repo's profile whether or not THIS shell
# exports it: the verdict below is about what the running nodes got, and the
# terminal running this script has nothing to do with that.
PROFILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-conf/fastdds_large_msg.xml}"
[ -r "$PROFILE" ] && want=$(grep -oP '(?<=<segment_size>)[0-9]+' "$PROFILE" | head -1)

if [ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "profile:  $FASTRTPS_DEFAULT_PROFILES_FILE (exported in this shell)"
else
    echo "profile:  $PROFILE (this shell does not export it -- only matters for"
    echo "          nodes started FROM this shell, not for the check below)"
fi
[ -n "$want" ] && printf "          segment_size = %d B (%d KB)\n" "$want" "$((want / 1024))"
printf "message:  CameraFrame = %d B (%d KB)\n\n" "$MSG" "$((MSG / 1024))"

printf "%10s %10s  %-8s %s\n" "SIZE_KB" "RAM_KB" "FRAMES" "SEGMENT / OWNER"

total=0
found=0
for f in /dev/shm/fastrtps_*; do
    [ -f "$f" ] || continue
    # *_el are the empty per-segment event-listener files, *_port* the small
    # shared port queues -- neither is a data segment, both are noise here.
    case "$f" in *_el|*port*) continue ;; esac

    sz=$(stat -c %s "$f")
    ram=$(du -k "$f" | cut -f1)
    total=$((total + ram))
    found=$((found + 1))

    # Every process that has it mapped, not just one: a reader mmaps the
    # writer's segment as well, so these are genuinely shared and naming a
    # single "owner" would be wrong.
    # A segment shared by many readers would print a whole roster, which is
    # noise -- name a few and count the rest. Bare pids are dropped when any
    # named process maps the same segment.
    names=$(for m in /proc/[0-9]*/maps; do
                grep -qF "$(basename "$f")" "$m" 2>/dev/null || continue
                pid=${m#/proc/}; pid=${pid%/maps}
                tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null \
                    | grep -oE '[a-z_]+_node|test_[a-z_]+|visionn_server' | head -1
            done | sort -u)
    n=$(echo "$names" | grep -c . || true)
    if [ "$n" -eq 0 ]; then
        owner="<no live owner>"
    elif [ "$n" -le 3 ]; then
        owner=$(echo "$names" | tr '\n' ' ')
    else
        owner="$(echo "$names" | head -3 | tr '\n' ' ')+$((n - 3)) more"
    fi

    printf "%10d %10d  %-8s %s  %s\n" \
        "$((sz / 1024))" "$ram" "$((sz / MSG))" "$(basename "$f")" "${owner:-<no live owner>}"
done

if [ "$found" -eq 0 ]; then
    echo "  (none -- no DDS participants running)"
else
    printf "\ntotal: %d segments, %d KB (%d MB) of RAM in /dev/shm\n" \
        "$found" "$total" "$((total / 1024))"
    # Each participant creates a small (~536 KB) segment alongside its data
    # segment, so a small one is NOT evidence of a node missing the profile --
    # an early version of this script warned on exactly that and was wrong.
    # What does mean the profile was missed is no segment reaching the size
    # the profile asks for while nodes are running.
    if [ -n "$want" ]; then
        big=0
        for f in /dev/shm/fastrtps_*; do
            case "$f" in *_el|*port*) continue ;; esac
            [ -f "$f" ] && [ "$(stat -c %s "$f")" -ge "$want" ] && big=$((big + 1))
        done
        if [ "$big" -eq 0 ]; then
            printf "\nWARNING: no segment reaches the profile's %d KB -- nodes were started\n         without FASTRTPS_DEFAULT_PROFILES_FILE. Re-source _setup_env.sh\n         in their terminal and restart them.\n" "$((want / 1024))"
        else
            printf "\n%d segment(s) at the profile size -- the large-message profile is in effect.\n" "$big"
        fi
    fi
fi
