# VM Setup / Installation Guide

Steps to bring a fresh VM to a state where `istrorsx_ws` builds and runs. Written for **Ubuntu 24.04 LTS (Noble Numbat)** — required, since it's the target platform for ROS 2 Jazzy.

---

## 1. Base OS Prerequisites
* Ubuntu 24.04 LTS, `amd64`.
* Enable the `universe` apt repository (ROS 2 packages depend on it):
  ```
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  ```
* Ensure a UTF-8 locale is active (`en_US.UTF-8` or equivalent) — required by ROS 2 tooling.

---

## 2. ROS 2 Jazzy
* Add the ROS 2 apt key and repository:
  ```
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  ```
* Install ROS 2:
  ```
  sudo apt update
  sudo apt install ros-jazzy-desktop
  ```
  (`ros-jazzy-desktop` is what this VM has installed; `ros-jazzy-ros-base` is a smaller alternative without GUI/visualization tools if that's ever preferred.)
* Source it in every shell (add to `~/.bashrc` for persistence):
  ```
  source /opt/ros/jazzy/setup.bash
  ```

---

## 3. Build Tooling
* `colcon` and `rosdep`:
  ```
  sudo apt install python3-colcon-common-extensions python3-rosdep
  sudo rosdep init
  rosdep update
  ```

---

## 4. `istrorsx_hw` Package System Dependencies
These are the non-ROS system libraries the package links against — install before the first `colcon build`.

* **Logging (`log4cxx`):**
  ```
  sudo apt install liblog4cxx-dev
  ```
* **Computer vision (`OpenCV`):** required by the build even though the ROS2-trimmed `ISTRO_ROS2` code path (see `01_architecture.md`) doesn't use most of it — the legacy sources still `#include` OpenCV headers.
  ```
  sudo apt install libopencv-dev
  ```
* **GPS daemon client (`gpsd`):** needed for `gps_node` (GPS hardware wrapper, ported from legacy `gpsdev.cpp`/`gpsdev.h`). The `ISTRO_GPSD_CLIENT` code path talks to a locally-running `gpsd` daemon over `libgpsmm`, not to the serial port directly. Run `script/install/gpsd_install.sh` (see § 6) rather than installing these by hand — it also disables the conflicting `gpsd.socket` systemd unit.
  * `libgps-dev` provides `libgpsmm.h`/`gps.h` and a `pkg-config` module (`libgps`, link flag `-lgps`) — it does **not** ship a CMake config package, unlike `log4cxx`. Any `CMakeLists.txt` target linking against it needs `find_package(PkgConfig REQUIRED)` + `pkg_check_modules(...)`.
  * Also installs `python3-gps` and `python3-serial`, used by the Python scripts in `script/` (device auto-detection, one-off GPS module configuration).
* **Geodesic math (`GeographicLib`):** needed for `gps_node`'s `GeoCalc` (`geocalc.cpp`/`geocalc.h`) — distance/azimuth between two lat/lon points (WGS84). Run `script/install/geolib_install.sh` (see § 6) rather than installing by hand.
  ```
  sudo apt install libgeographiclib-dev
  ```
  * Ships **both** a `pkg-config` module (`geographiclib`, link flag `-lGeographicLib`) *and* a CMake find-module (`FindGeographicLib.cmake`) — this package uses the `pkg-config` route via `pkg_check_modules(...)`, for consistency with how `libgps` is discovered (which has no CMake config option at all).
  * The legacy project used to build GeographicLib 1.46 from source (`script/install/geolib_install.sh`'s original, commented-out content) — not needed on Ubuntu 24.04, which packages a current version (2.3) directly.
* **Lidar SDK (`rplidar_sdk`):** needed for the planned `lidar_node` (GPS/ctrlboard-style wrapper around the legacy `lidar_rplidar.cpp`/`lidar.h`, not yet ported). Unlike `libgps`/`libgeographiclib`, there is no apt package or `pkg-config`/CMake config for this SDK — it's built from source via `script/install/rplidar_install.sh` (see § 6) into a fixed system path.
  * Source: [github.com/Slamtec/rplidar_sdk](https://github.com/Slamtec/rplidar_sdk) (current v2.x), cloned to `/usr/local/rplidar_sdk` and built with `make -C sdk`, producing a static library at `/usr/local/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a` and headers under `/usr/local/rplidar_sdk/sdk/include/`.
  * The legacy project used SDK v1.6.1/v1.11.0 (`script/install/rplidar_install.sh`'s original, commented-out content — a dead download URL) with the old free-standing `RPlidarDriver`/`IS_OK`/`IS_FAIL`/`rplidar_response_measurement_node_t` API. The current SDK is a namespaced rewrite (`sl::createLidarDriver()`, `SL_IS_OK`/`SL_IS_FAIL`, `sl_lidar_response_measurement_node_hq_t`, channel-based `connect()`) — porting `lidar_rplidar.cpp` will need to target this new API, not just a recompile.
  * Since the SDK ships no `pkg-config`/CMake config, `lidar_node`'s `CMakeLists.txt` will need to reference `/usr/local/rplidar_sdk/sdk/include` and the static library path directly (same pattern the legacy build used for `/usr/local/rplidar/sdk/`).
* **RealSense camera (`librealsense2`):** needed for the planned `camera_node` (wrapper around the legacy `camera.cpp`/`camera.h`, not yet ported), which uses `<librealsense2/rs.hpp>` (`rs2::pipeline`/`rs2::context`/`rs2::config`) and links `-lrealsense2`. Run `script/install/realsense_install.sh` (see § 6) rather than installing by hand.
  ```
  sudo apt-get install -y librealsense2-utils librealsense2-dev librealsense2-dbg
  ```
  * Installed from Intel/RealSense AI's own signed apt repo (`librealsense.realsenseai.com` — RealSense was spun off from Intel into its own company, replacing the older `librealsense.intel.com`/`realsense.intel.com` repos referenced in the legacy notes), not built from source — much lighter on this VM's limited RAM (3.7 GiB, no swap) than the from-source `-DFORCE_RSUSB_BACKEND=true` build documented as a fallback in `script/install/realsense_install.sh` (only needed if `realsense-viewer` ends up missing after the apt install, a known issue on some arm64 targets; not needed on this VM — verified present).
  * Ships a proper CMake config package (`realsense2Config.cmake`) — unlike `libgps`, a future `camera_node`'s `CMakeLists.txt` can use `find_package(realsense2 REQUIRED)` directly, no `pkg-config` needed.
  * Verify hardware detection (requires an attached RealSense camera): `rs-enumerate-devices` or `realsense-viewer`.
* **QR/barcode scanning (`ZBar`):** needed for the planned `vision_node` (`istrorsx_core` — its `QRScanner`, ported from legacy `qrscan.cpp`/`qrscan.h`), which `#include <zbar.h>` and links `-lzbar`. Run `script/install/zbar_install.sh` (see § 6) rather than installing by hand.
  ```
  sudo apt-get install -y libzbar-dev libzbar0
  ```
  * Ships a `pkg-config` module (`zbar`) — no CMake config package, same discovery route as `libgps`/`GeographicLib` (`find_package(PkgConfig REQUIRED)` + `pkg_check_modules(...)`).
  * Unchanged from the legacy project's own install note (`script/install/zbar_install.sh`'s original content) — still current on Ubuntu 24.04/ROS 2 Jazzy.

---

## 5. Fetching & Building the Workspace
* Clone/copy the workspace so that `istrorsx_hw` ends up under `<workspace_root>/src/istrorsx_hw`.
* From `<workspace_root>`:
  ```
  colcon build --packages-select istrorsx_hw
  source install/setup.bash
  ```
* Re-run `source install/setup.bash` (or open a new shell that sources it) whenever you rebuild.
* Prefer `source _setup_env.sh` in the workspace root over sourcing `install/setup.bash` directly: it
  sources both ROS and the overlay, and also exports `FASTRTPS_DEFAULT_PROFILES_FILE`
  (`conf/fastdds_large_msg.xml`, which sizes Fast DDS's shared-memory segment for the 1.74 MB
  `CameraFrame`). A node started from a shell without that variable silently keeps the 512 KB
  default — nothing fails, it is just the stock configuration. `script/test/test_dds_shm.sh` says
  which one a running system actually got. The `*_run.sh` node scripts source `_setup_env.sh`
  themselves, so they are correct from any shell.

---

## 6. Runtime Layout & GPS Daemon Configuration
These matter when actually **running** the node(s), not just building.

* **Working directory matters.** `istro_main_ctrlboard()` (and `istro_main_gps()`, once `gps_node` exists) loads logging config from a path relative to the process's current working directory: `conf/log4cxx.xml`. That file's rolling-file appender writes to `logout/istro_rt2026.*.log`, also a relative path. Both `conf/` and `logout/` must exist **relative to wherever the node is launched from** — at the workspace root in this project (see `conf/log4cxx.xml`, `ctrlboard_node_run.sh`). Running from a different directory without these subfolders present will fail to configure logging.
* **`<workspace_root>/script/` holds the GPS operational scripts**, ported from the legacy `doc/istrobtx_2025/script/` (kept as read-only reference — treat `script/` as the maintained copy). Run these **before starting the robot**, not as part of the build:
  * `install/gpsd_install.sh` — one-time: installs `gpsd`, `gpsd-clients`, `libgps-dev`, `python3-gps`, `python3-serial`, and disables the conflicting `gpsd.socket` systemd unit (see § 4 above). Safe to re-run (idempotent).
  * `install/geolib_install.sh` — one-time: installs `libgeographiclib-dev` (see § 4 above). Safe to re-run (idempotent).
  * `install/rplidar_install.sh` — one-time: clones/updates and builds `rplidar_sdk` under `/usr/local/rplidar_sdk` (see § 4 above). Safe to re-run (idempotent — `git pull` + `make` are no-ops once up to date).
  * `install/realsense_install.sh` — one-time: adds the RealSense AI apt repo and installs `librealsense2-utils`/`librealsense2-dev`/`librealsense2-dbg` (see § 4 above). Safe to re-run (idempotent).
  * `install/zbar_install.sh` — one-time: installs `libzbar-dev`/`libzbar0` (see § 4 above). Safe to re-run (idempotent).
  * `gpsd_boot.sh` — auto-detects the GPS USB-serial adapter via `udevadm` (matches on `Prolific_Technology_Inc._USB-Serial_Controller_D`), sends the module's NMEA config commands, and starts `gpsd` bound to it. Prefer this one when only one recognized GPS dongle is attached.
  * `gpsd_start.sh` — same idea, but binds `gpsd` to the `/dev/ttyUSB_GPS` udev symlink directly. It used to pick the device via `gps_port.py` (Python/pyserial, VID:PID match); that line is kept commented in the script, since VID:PID matching is the fallback on a machine without the udev rule.
  * `gpsd_start_USB0.sh` / `_USB1.sh` / `_USB3.sh` — same, but with the device hardcoded to a specific `/dev/ttyUSBx` — use when auto-detection picks the wrong device (e.g. multiple USB-serial adapters attached).
  * `gpsd_stop.sh` — stops `gpsd` cleanly, and the `gpspipe` recording with it.
  * `gpsd_gpspipe.sh` — dumps the raw `gpsd` JSON stream (`gpspipe -r`) for debugging.
  * `_gpsd_run.sh` (workspace root, run manually before `_full_launch.sh`) — **the normal way to bring the GPS up**: stops `gpsd`, runs `gpsd_init.sh`, starts `gpsd`. `set -e` means a module that fails verification aborts the script and `gpsd` is never started, rather than the fault surfacing later while the robot is driving.
    It then records the raw NMEA stream for the whole session in the background: `gpspipe -r >> logout/gpspipe.log` (legacy `r.sh` did the same into `out/gpspipe.log`). `gpspipe` comes from `gpsd-clients`, which `script/install/gpsd_install.sh` already installs. The recording is appended, not written with `-o`, so `script/mvout.sh` can copy it into the run archive and truncate it while it keeps running.
  * `gpsd_init.sh` — one-off GPS module configuration (restrict NMEA output to RMC sentences, disable the nav-speed threshold) — not part of the normal start sequence, only needed once per fresh GPS module or after a factory reset. Run it with `gpsd` stopped, or the writes never reach the module. It is a thin wrapper around `gpsd_init.py`, which finds the GPS by USB VID:PID rather than by device node and prints the module's replies to each command. After the last write it **verifies** for 6 s that the module really does what it was told: at least 4 RMC sentences, no other NMEA sentence (`PMTK314` took effect), valid checksums, and a `$PMTK527` reply of `0.00` (`PMTK386` took effect). It exits non-zero on any failure, and also when no GPS is found at all. The `.py` deliberately has neither a shebang nor the execute bit: `gpsd_init.sh` is the single entry point, and the interpreter is named in exactly one place.
  * `gpsd_init_old.sh` — the previous shell-only implementation of the same thing, superseded but kept: it needs nothing beyond pyserial's `miniterm`, and it ends in an interactive terminal on the GPS, which is the only way to type the `$PMTK447*35` check by hand or to send a command the pair above doesn't cover.
  * `gps_port.py` — device auto-detector (VID:PID `067B:2303`, Prolific PL2303) used by the scripts above; prints `/dev/null` if no matching device is found.
  * Portability note: the originals used the Ubuntu-14.04-era `python`/`miniterm.py` commands. On this distro only `python3` exists, and pyserial's tool is named `pyserial-miniterm` — `gpsd_start.sh` and `gpsd_init_old.sh` were updated accordingly (see the `NOTE` comments in those files); `gpsd_boot.sh` never depended on Python and needed no change.
* **`gpsd` must point at the real GPS serial device.** The default systemd socket-activation setup (`gpsd.socket`, enabled automatically by the `gpsd` package) does not know which device to watch — `install/gpsd_install.sh` disables it, and one of the `gpsd_start*.sh` / `gpsd_boot.sh` scripts above must be run to actually start `gpsd` pointed at the device. `gps_node` will only get real fixes if `gpsd` is already running this way — it does not start or configure `gpsd` itself. Without `gpsd` running, `gps_node`'s `istro_main_gps()` fails at `gps.init()`, and `main()` exits immediately with failure instead of spinning (see `01_architecture.md` § Init / Shutdown Split) — start `gpsd` first.
* **Example launch invocation** (from `ctrlboard_node_run.sh`, workspace root as CWD):
  ```
  ros2 run istrorsx_hw ctrlboard_node -- -nowait -cb /dev/null -cb2 /dev/null -lidar /dev/null -i DHELLO -path E3 -vf 7 -vf2 14 -vf3 21 -cg 300 -ca 0
  ```
  The `--` separates ROS 2's own arguments from the legacy `argc`/`argv`-style flags consumed by `Config::parseArguments()` (see `01_architecture.md` § Startup / CLI Arguments). `/dev/null` here is a placeholder device for running without real hardware attached — swap in the real serial device path (e.g. `/dev/ttyUSB0`) on an actual robot. `gps_node` takes no device-path flag (see `01_architecture.md` § Startup / CLI Arguments) — minimally, `ros2 run istrorsx_hw gps_node -- -nowait`.
