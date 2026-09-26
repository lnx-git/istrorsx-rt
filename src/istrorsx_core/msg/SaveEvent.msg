# save_node's own "what did I just write to disk" notification -- lets
# telemetry_node learn about newly-saved files asynchronously instead of
# tailing logout/istro_<EVENT_TAG>.log for saveImage() lines (legacy logp.cpp's
# current approach, kept as telemetry_node's near-term fallback/bootstrap --
# see doc/ai/01_architecture.md's telemetry_node section). Published once
# per save_node batch -- mirrors legacy save_thread's own save_change
# bitmask grouping, where several files get written together in one
# throttled cycle (e.g. camera+vision+nnpred+cdepth all at once, or
# lidar+wmgrid[+wmgrif][+navmap] all at once), not one message per file.
#
# A flat "one string field per known image tag" design (mirroring legacy
# logp.cpp's own JSON shape) was considered and rejected: it can't represent
# SaveData.msg's per-entry image_number correctly (wmgrid/wmgrif use a
# different clock than the rest of the batch, see that message's own
# comment), and it would need a new field added here every time save_node
# gains a new image tag. An array handles both for free.
#
# files[] contains ONLY the tags actually (re)written in this batch, not one
# entry per known tag every time -- legacy save_thread itself never saves
# everything on every iteration (per-group throttle flags, some inputs may
# be empty/unavailable this tick, wmgrif/navmap only every 10s/1s). A
# consumer (telemetry_node) wanting "what is the latest known file for every
# tag right now" must accumulate its own tag -> filename map across
# messages, updating only the tags present in each one -- the exact same
# pattern legacy logp.cpp's own findResult[] array already does today
# (remembers the last match per pattern, only overwritten when a newer one
# appears), just fed by these messages instead of by parsing log lines.

SaveData[] files
