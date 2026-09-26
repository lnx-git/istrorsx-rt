# Raw GPS logs — Robotour 2026, Praha Stromovka, 2026-09-19

Every NMEA sentence the receiver produced during the competition, exactly as it came out of the
module. Recorded with `gpspipe -r` alongside the ROS nodes (see `_gpsd_run.sh` in the repository
root), one file per run.

Receiver: **Holux M-215+** (GPS/GLONASS, USB serial, 4800 Bd), read through `gpsd`. It is configured
at start-up to emit **RMC sentences only** and with the navigation speed threshold switched off
(`PMTK314` and `PMTK386,0`, see `script/gpsd_init.py`) — without that, a slow or standing robot gets
no position updates at all. So these files are a plain stream of `$GNRMC` lines at 1 Hz.

Times in the file names are the end of each run, matching how the runs are named in the team's own
archive; times in the table are the first and last valid fix in the file itself, local time (CEST,
UTC+2). The rounds ran on the south-east part of Stromovka — the map is
`conf/navmap-praha-stromovka-jv.osm`.

| File | Round | Fixes | Covers | The run |
|---|---|---|---|---|
| `260919_0900_preround_gpspipe.log.gz` | pre-round | 1 770 | 09:12–09:43 | qualification; the NN model was changed after the load |
| `260919_1034_round_1a_gpspipe.log.gz` | 1 | 2 504 | 09:53–10:34 | the robot locked up in a WrongWay reversal |
| `260919_1046_round_1b_gpspipe.log.gz` | 1 | 786 | 10:34–10:47 | return to the service area |
| `260919_1207_round_2a_gpspipe.log.gz` | 2 | 2 809 | 10:47–12:07 | the robot left the map |
| `260919_1225_round_2b_gpspipe.log.gz` | 2 | 1 086 | 12:07–12:25 | return to the service area |
| `260919_1438_round_3_gpspipe.log.gz` | 3 | 2 538 | 12:25–14:39 | off the path during a WrongWay reversal |
| `260919_1610_round_4_gpspipe.log.gz` | 4 | 1 990 | 15:32–16:10 | trouble in the map; the Jetson shut down on low UPS voltage |

The first file also contains sentences from before the first fix — the module reports `V` (invalid)
with its default date until it acquires satellites; the fix count above counts only valid (`A`)
sentences.

Reading one of them:

```bash
zcat 260919_1034_round_1a_gpspipe.log.gz | head
```
