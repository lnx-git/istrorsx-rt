# Specifications & ROS Interfaces

Reference for **what exists**: packages, modules, nodes, messages, topics. Terse by design — the
*why* behind any of it lives in `01_architecture.md`, status/history in `03_progress.md`.

---

## 1. Packages

| Package | Type | Contains |
|---|---|---|
| `istrobtx` | plain C++ library (no ROS interfaces) | shared legacy scaffold: `Config`, `DataSet`, `Threads`, `Logger`, `mtime`, `system`, `GeoCalc` + hardware-independent `*_defs.h` extractions |
| `istrorsx_hw` | ROS 2, 4 nodes + 9 msgs | hardware wrappers: control board, GPS, lidar, RealSense cameras |
| `istrorsx_core` | ROS 2, 7 nodes + 10 msgs + 1 srv | perception/navigation/decision/persistence logic ported from `istro_rt2025.cpp` |
| `istrorsx_bringup` | ROS 2, launch only (no code) | `full.launch.py` — brings the whole system up at once |

Dependency direction: `istrorsx_core` → `istrorsx_hw` → `istrobtx`. **Never the reverse** — this is why
`ImageNumber.msg` exists in `istrorsx_hw` (see its note in §3).

### `istrobtx` modules (one line each)

| Module | Role |
|---|---|
| `config.h/.cpp` | legacy CLI parsing (`-cb`, `-lidar`, `-path`, …) + all runtime flags/velocities |
| `dataset.h/.cpp` | legacy shared-memory struct passed between threads; mostly vestigial here |
| `threads.h/.cpp` | legacy pthread buffer-slot synchronizer; mostly vestigial here |
| `logger.h` | `LOG_DEFINE`/`LOGM_*` macros over log4cxx |
| `mtime.h/.cpp` | `timeBegin()`/`timeEnd()`/`timeDelta()` timing + `mtime::` TRACE logging |
| `system.h` | compile-time feature switches (`ISTRO_ROS2`, `ISTRO_VISIONN`, `ISTRO_MAP_*`, …) |
| `geocalc.h/.cpp` | WGS84 distance/azimuth between lat/lon pairs (GeographicLib) |
| `dmap.h/.cpp` | `DegreeMap` — 181-entry angular free/obstacle map, the common perception currency |
| `event_defs.h` | `EVENT_TAG` — competition file-naming tag (`"rt2026"`), single source of truth |
| `lidar_defs.h/.cpp` | `lidar_data_t` + calibration constants + `lidar_process()`/`lidar_draw_output()` |
| `camera_defs.h/.cpp` | depth-frame constants + `camera_draw_depth_frame()` |
| `ctrlboard_defs.h/.cpp` | `SA_*`/`VEL_*`/`CTRLB_STATE_*` constants + `ctrlb_obstState()` |

### `istrorsx_hw` / `istrorsx_core` modules

Each node has its own `<node>.cpp/.hpp` plus an `istro_main_<subsystem>.cpp/.h` init scaffold
(`Config`/`Threads`/`DataSet` setup, `initDevices()`/`closeDevices()`) — **except `telemetry_node`**,
which needs no scaffold. Hardware drivers and ported legacy algorithm modules:

| Module | Package | Role |
|---|---|---|
| `ctrlboard.cpp` | hw | serial protocol with the control board |
| `gpsdev.cpp` | hw | `gpsd` client (`libgpsmm`) |
| `lidar.cpp` | hw | RPLIDAR SDK wrapper |
| `camera.cpp` | hw | RealSense (`librealsense2`) capture |
| `vision.cpp`, `sample.cpp` | core | color-based road/off-road classification + calibration samples |
| `vision_depth.cpp` | core | depth-image obstacle detection |
| `visionn.cpp` | core | TCP client to the external NN inference server (port 7001) |
| `qrscan.cpp` | core | QR code scanning (ZBar) |
| `wmodel.cpp` | core | `WorldModel`/`WMGrid` — fused occupancy grid |
| `navig.cpp`, `navig_data.cpp` | core | navigation points, local XY conversion |
| `navmap.cpp`, `navmap_data.cpp` | core | route graph (OSM-derived), routing, KML/PNG export |

---

## 2. Nodes

| Node | Package | Legacy origin | Role |
|---|---|---|---|
| `ctrlboard_node` | hw | `ctrlBoard_thread` | motor/servo/LED commands out, IMU+servo telemetry in (5 ms tick) |
| `gps_node` | hw | `gps_thread` | GPS fixes via `gpsd` (200 ms tick) |
| `lidar_node` | hw | `capture_lidar_thread` | raw 360° scans |
| `camera_node` ×2 | hw | `capture_camera_thread` | RealSense color+depth; one process per camera (`-camdev 0`/`1`) |
| `vision_node` | core | `vision_thread` | color+NN+depth+QR perception → free/obstacle angular map |
| `navigation_node` | core | `gps_thread`'s nav extension | route matching, next navigation point, local XY |
| `planner_node` | core | `process_thread` | fuses everything → the actual driving decision |
| `drive_node` | core | `loop()` | arbitration: manual (Joy) vs autonomous, obstacle priority |
| `keyboard_node` | core | X11 `cv::waitKey()` | terminal keyboard → `Joy` (manual control without a gamepad) |
| `save_node` | core | `save_thread` | throttled debug images/maps to `out/` |
| `telemetry_node` | core | `script/logp.cpp` | tails the shared log → `ramdisk/*.{txt,html,json}` for the dashboard |

---

## 3. Messages — `istrorsx_hw`

**`SpeedCommand.msg`** — `bool trigger_start`, `trigger_stop`, `set_steering_angle`, `int32 steering_angle`, `bool set_speed`, `int32 speed`.

**`LedConfig.msg`** — index control (`set_led_index`, `led_index`, `led_index_r/g/b`, `led_index_blink`), mask control (`set_led_mask`, `led_mask`, `led_mask_r/g/b`, `led_mask_blink`), program control (`set_led_program`, `led_program`).

**`DeviceAction.msg`** — `display_text`+`display_text_data`, `write_string`+`write_string_data` (raw serial), `set_xx4`, `set_xx5`, `set_ball_drop`+`ball_drop_cnt`.

**`ImuData.msg`** — `float64 euler_x/y/z`, `int32 calib_gyro/accel/mag`.

**`ServoData.msg`**
* `int32 state` (1=STOP, 2=FWD, 5=OBST), `float64 heading` (unused — board firmware doesn't populate it)
* `int32 ircv` (encoder pulses this interval), `float64 ircv500` (sum over last 500 ms)
* `int32 angle` (steering setting), `velocity` (motor setting), `loadd` (load detection), `cbtime` (board ms)
* `int32 ulsd1`…`ulsd5` — ultrasonics in cm: left, middle/front, right, rear, rear
* Field names match `ControlBoard::getServoData()`'s parameters — **not** the `ctrlb_`-prefixed `DataSet` member names.

**`GpsData.msg`** — `bool fix`, `float64 latitude`/`longitude` (deg), `speed` (m/s), `course` (deg, true north).
Published every tick regardless of fix; `fix` itself is the validity signal.

**`LidarData.msg`** — `int64 image_number` (stamped from `/robot/image_number`, same mechanism as `CameraFrame.msg`, so `save_node` tags `lidar.png` with the scan's own number) + `int32 point_count` + parallel arrays `int32[] sync` (1 = first sample of a new 360° scan), `float32[] angle` (deg, 0=right/90=front, CCW), `float32[] distance` (mm, 0 = no reading), `int32[] quality` (0–63). Angle-ascending, mirrors `lidar_data_t`.

**`CameraFrame.msg`** — color+depth from one capture instant, in one message.
* `color_*` (width/height/cv_type/step/data): BGR8, `cv_type`=`CV_8UC3`=16, always present
* `depth_*`: millimetres, `cv_type`=`CV_16UC1`=2; `depth_width == 0` = depth unavailable this frame
* `int64 image_number` — stamped from `/robot/image_number` (see below)
* `*_cv_type` is the literal `cv::Mat::type()`, not a `sensor_msgs` encoding string — consumers rebuild a `Mat` directly, no `cv_bridge`.

**`ImageNumber.msg`** — `int64 image_number`. `planner_node`'s virtual clock, republished in `istrorsx_hw`
purely so `camera_node` can stamp frames with it: `istrorsx_core` depends on `istrorsx_hw`, so
`camera_node` cannot subscribe to `istrorsx_core`'s `PlannerData.msg` without a circular dependency.

**`UpsData.msg`** — Waveshare UPS Power Module (C) battery telemetry (INA219 over I2C), no legacy
equivalent. `float64 bus_voltage` (V, load-side), `shunt_voltage` (V), `current` (A), `power` (W),
`percentage` (0–100, estimated from `bus_voltage`). Published by `ups_node` every `poll_period_ms`
(default 2000ms); no current subscriber.

---

## 4. Messages — `istrorsx_core`

**`VisionData.msg`** — `int32 camera_id` (0=front, 1=rear), `int64 image_number`, `int32[181] dmap` (1=free, 0=obstacle, −1=n/a), `int32[181] dist` (cm, <0 = none), `int32[181] maxd` (cm), `int32 angle_min`/`angle_max` (widest free interval, −1 = none), `float64 qrscan_latitude`/`qrscan_longitude`.

**`VisionDebugData.msg`** — same `camera_id`/`image_number`/`angle_min`/`angle_max`, plus four
`cv::Mat`-as-bytes field groups (`*_width`/`_height`/`_cv_type`/`_step`/`_data`): `markers`, `epweight`,
`elweight`, `camera_img_pred`. Consumed only by `save_node` for debug overlays — split from `VisionData`
so the driving-critical path doesn't carry megabytes.

**`VisionControl.msg`** — `bool qrscan_enabled`, `bool process_rear_enabled`. Mode control for
`vision_node`; port of legacy's process-global `vision_thread_*_enabled` flags.

**`NavigationData.msg`**
* `int32 navp_idx` (next route point, −1=none), `float64 navp_dist`/`navp_dist_raw`, `navp_azimuth`/`navp_azimuth_raw` (route-corrected / straight-line GPS), `navp_maxdist`
* `int32 navp_loadarea` (`NONE=-1`, `LOADING=0`, `UNLOADING=1`, `BALLDROP=10`), `float64 navp_latitude`/`longitude` + `_raw` variants
* `int32 ref` (1 = reference point initialized), `float64 x`/`y` (**GPS-derived** local XY metres)
* Mirrors `DataSet`'s `gps_navp_*`/`gps_ref`/`gps_x`/`gps_y` with the prefix dropped.

**`NavigationPointSet.msg`** — `int32 point_idx`, `float64 point_latitude`/`point_longitude`.
Cross-process replacement for legacy's direct `navigation_point_set()` call, when a QR-scanned
coordinate resolves a navigation point's real position. **Every process holding its own
`navigationPoint[]` copy must subscribe** — legacy had one shared array, this port has three separate
ones (`navigation_node`, `planner_node`, `save_node` all link `navig_data.cpp`). Currently applied by
`navigation_node` (route matching) and `save_node` (draws the array in `navmap.png`/`.kml`);
`planner_node` publishes but needs no local copy — it never reads `navigationPoint[]`, it gets
navigation state back via `NavigationData.msg`.

**`NavigationRoute.msg`** — `int32[] route_segment_indices`. The currently-planned route, as indices
into `navMapSegment[]` (`navmap_data.cpp`), published every `navigationTick()`. Same category of gap
as `NavigationPointSet.msg` above: `NAVMAP_FLAG_ROUTE` is **live, mutable, per-process** state set by
`navmap_planRouteLL()` inside `navigation_node` only, so `save_node`'s separately-linked
`navMapSegment[]` copy never had it and `navmap.png`/`.kml` route highlighting was always empty.
Indices are portable because the array's base topology is compile-time-identical everywhere —
`NAVMAP_FLAG_TMP_PLAN` segments (the temporary start/end connectors, which exist only in
`navigation_node`'s live-grown array) are filtered out before publishing, and `save_node`
bounds-checks each index anyway.

**`PlannerData.msg`** — secondary planner output (what `DriveCommand` doesn't cover)
* `int64 image_number` — virtual clock, +1 per `plannerTick()`; the cross-node log/file correlation key
* `int32 process_dir` (steering preference, 90=straight), `process_ref`, `float64 process_x`/`process_y` (**odometry** dead-reckoning — deliberately keeps the `process_` prefix so it can't be confused with `NavigationData`'s GPS-derived `ref`/`x`/`y`), `int32 process_backward`
* `int32 coneseek_stop`/`coneseek_intlen` — `ISTRO_VISION_ORANGECONE` mode only

**`DriveCommand.msg`** — the driving decision
* `int32 ANGLE_OK=999998` (constant) — `angle` is only acted on if `angle < ANGLE_OK`
* `int32 angle`, `velocity` (−1 = not set), `state` (`PROCESS_STATE_*`), `stop` (<0 = not set, 0 = no stop, >0 = stop), `float64 yaw`

**`PlannerDebugData.msg`** — `int64 image_number` (same tick as `PlannerData`), `int32 lidar_angle_min`/`lidar_angle_max`/`lidar_stop` (raw lidar), `process_angle_min`/`process_angle_max` (fused `wmodel` grid). For `save_node`'s lidar overlay.

**`SaveData.msg`** — `string tag` (`camera`/`vision`/`nnpred`/`cdepth`/`rcamera`/`rvision`/`rnnpred`/`rcdepth`/`lidar`/`wmgrid`/`wmgrif`/`navmap_png`/`navmap_kml`), `string filename` (e.g. `out/rt2026_0002345_0000042_camera.jpg`), `int64 image_number`. Not published standalone — only nested in `SaveEvent`.

**`SaveEvent.msg`** — `SaveData[] files`. **Incremental** notification: only the tags actually (re)written
this batch, not a full-state snapshot. Per-entry `image_number` (not one shared field) because
`wmgrid`/`wmgrif` are stamped with `wmodel`'s own slower clock than the rest of a batch.

---

## 5. Service

**`GetWMGridSnapshot.srv`** (`istrorsx_core`) — empty request; response = the `~4 MB` occupancy grid
(`grid_width`/`height`/`cv_type`/`step`/`data`, `grid_x0`/`grid_y0`) + `image_number` and the robot pose
at that update (`last_x0`/`last_y0`/`last_alfa`/`last_ref`/`last_angle`/`last_angle_min`/`last_angle_max`).

Served by `planner_node`, called by `save_node` **only when it has decided to render** — pull, not push,
so the grid's serialization cost is paid on the consumer's schedule. `save_node` must call it
asynchronously (single-threaded executor).

**`NavMapExport.srv`** (`istrorsx_core`), served by `navigation_node` at `/robot/navmap_export` — port of
legacy `navmap_test()`. Request `map_name` (e.g. `navmap-ba-sadjk`, trailing `.osm` accepted); response
`success`, `message` (`"ok"` or the error), `osm_file` (`conf/<map>.osm`), `kml_file` (`out/<map>.kml`),
`cpp_file` (`out/<map>.cpp`), `node_cnt`/`segment_cnt` loaded from the OSM file; connectivity of the loaded graph:
`component_cnt` (1 = connected), `main_node_cnt`/`main_segment_cnt` (largest component), `disconnected_way_ids`/
`disconnected_node_ids` (OSM ids outside it). The live navigation map is restored after the export.

---

## 6. Topic map

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `/robot/servo_data` | `ServoData` | `ctrlboard_node` | `drive_node`, `planner_node` |
| `/robot/imu_data` | `ImuData` | `ctrlboard_node` | `planner_node` |
| `/robot/gps_data` | `GpsData` | `gps_node` | `drive_node`, `navigation_node`, `planner_node`, `save_node` |
| `/robot/lidar_data` | `LidarData` | `lidar_node` | `planner_node`, `save_node` |
| `/robot/camera_front_data` | `CameraFrame` | `camera_node` (`-camdev 0`) | `vision_node`, `save_node` |
| `/robot/camera_rear_data` | `CameraFrame` | `camera_node` (`-camdev 1`) | `vision_node`, `save_node` |
| `/robot/image_number` | `ImageNumber` | `planner_node` | `camera_node` |
| `/robot/ups_data` | `UpsData` | `ups_node` | *(none yet)* |
| `/robot/vision_front_data` | `VisionData` | `vision_node` | `planner_node` |
| `/robot/vision_rear_data` | `VisionData` | `vision_node` | `planner_node` |
| `/robot/vision_front_debug_data` | `VisionDebugData` | `vision_node` | `save_node` |
| `/robot/vision_rear_debug_data` | `VisionDebugData` | `vision_node` | `save_node` |
| `/robot/vision_control` | `VisionControl` | `planner_node` | `vision_node` |
| `/robot/navigation_data` | `NavigationData` | `navigation_node` | `planner_node`, `save_node` |
| `/robot/navigation_point_set` | `NavigationPointSet` | `planner_node` | `navigation_node`, `save_node` |
| `/robot/navigation_route` | `NavigationRoute` | `navigation_node` | `save_node` |
| `/robot/planner_data` | `PlannerData` | `planner_node` | `vision_node`, `navigation_node` |
| `/robot/planner_debug_data` | `PlannerDebugData` | `planner_node` | `save_node` |
| `/robot/drive_command` | `DriveCommand` | `planner_node` | `drive_node`, `save_node` |
| `/robot/speed_command` | `SpeedCommand` | `drive_node` | `ctrlboard_node` |
| `/robot/led_config` | `LedConfig` | `drive_node` | `ctrlboard_node` |
| `/robot/device_action` | `DeviceAction` | `drive_node` | `ctrlboard_node` |
| `/robot/save_event` | `SaveEvent` | `save_node` | *(none yet — for `telemetry_node` round 2)* |
| `/joy` | `sensor_msgs/msg/Joy` | `joy_node` (std pkg) or `keyboard_node` | `drive_node` |

`/joy` is deliberately **not** `/robot/joy` — it matches the standard `joy` package's default, the one
narrow exception to this project's "no external ROS 2 packages" rule.

`telemetry_node` has **no** ROS interfaces at all (round 1 tails the log file directly).

---

## 7. Known interface gaps

* **`/robot/save_event` has no subscriber yet** — by design; it exists for `telemetry_node`'s planned
  round 2 (online mode fed by messages instead of log tailing).
* *(Fixed 2026-08-30, was a real gap: `/robot/navigation_point_set` had no subscriber at all — the
  QR-scan coordinate update was inert end-to-end. `navigation_node` and `save_node` both subscribe
  now; see §4's own note on why all three `navigationPoint[]`-holding processes matter.)*
* `camera_node` also handles a `/robot/camera_data` fallback topic name, used only when `-camdev`
  is neither 0 nor 1.
