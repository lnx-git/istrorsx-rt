cmake_minimum_required(VERSION 3.10)
project(istrorsx_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(log4cxx REQUIRED)
find_package(OpenCV REQUIRED)

# Shared legacy scaffold (Config, DataSet, Logger, Threads, mtime, system) --
# split out into its own package since it's identically needed by every node
# here AND by istrorsx_core's drive_node. See doc/ai/01_architecture.md.
find_package(istrobtx REQUIRED)

# libgps (gpsmm, for GpsDevice) ships no CMake config package (unlike
# log4cxx) -- discovered via pkg-config instead. GeographicLib (for GeoCalc)
# used to be found here too, but GeoCalc moved into the shared istrobtx
# package (also needed by istrorsx_core's navigation_node), which now finds
# and exports it itself -- see doc/ai/01_architecture.md.
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPS REQUIRED libgps)

# rplidar_sdk ships neither a CMake config package nor a pkg-config module --
# built from source by script/install/rplidar_install.sh into a fixed system
# path (see doc/ai/04_installation.md), referenced directly here. Only
# lidar_node needs this now: DataSet used to pull in the full lidar.h (and
# therefore the RPLIDAR SDK) transitively for every target, but lidar.h's
# lidar_data_t/LIDAR_DATA_NUM were split out into istrobtx's SDK-independent
# lidar_defs.h, so ctrlboard_node/gps_node/camera_node no longer need this
# include path at all -- see doc/ai/03_progress.md.
set(RPLIDAR_SDK_DIR /usr/local/rplidar_sdk)
set(RPLIDAR_SDK_INCLUDE_DIR ${RPLIDAR_SDK_DIR}/sdk/include)
set(RPLIDAR_SDK_LIBRARY ${RPLIDAR_SDK_DIR}/output/Linux/Release/libsl_lidar_sdk.a)

# librealsense2 (for Camera), unlike libgps/rplidar_sdk, ships a proper CMake
# config package (installed by script/install/realsense_install.sh from the
# RealSense AI apt repo, see doc/ai/04_installation.md) -- discovered normally.
find_package(realsense2 REQUIRED)

find_package(rosidl_default_generators REQUIRED)
#find_package(rosidl_cmake REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SpeedCommand.msg"
  "msg/LedConfig.msg"
  "msg/DeviceAction.msg"
  "msg/ImuData.msg"
  "msg/ServoData.msg"
  "msg/GpsData.msg"
  "msg/LidarData.msg"
  "msg/CameraFrame.msg"
  "msg/ImageNumber.msg"
  "msg/UpsData.msg"
)

find_package(rclcpp REQUIRED)

#rosidl_get_typesupport_target(cpp_typesupport_target
#  ${PROJECT_NAME} rosidl_typesupport_cpp)

#rosidl_target_dependencies(ctrlboard_node
#  "${PROJECT_NAME}"
#)

#add_dependencies(ctrlboard_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Explicit source list (not a glob) so each executable only pulls in --
# and links against -- what it actually needs. gpsdev.cpp is intentionally
# excluded here: it needs libgps (gpsmm) linkage that ctrlboard_node doesn't
# have, and is built into the separate gps_node executable instead.
# config.cpp/dataset.cpp/logger.cpp/mtime.cpp/system.cpp/threads.cpp moved to
# the istrobtx package (linked below), not listed here anymore.
set(CTRLBOARD_NODE_SOURCES
  src/ctrlboard_node.cpp
  src/istrobtx/ctrlboard.cpp
  src/istrobtx/istro_main_ctrlboard.cpp
)
add_executable(ctrlboard_node ${CTRLBOARD_NODE_SOURCES})
#ament_target_dependencies(ctrlboard_node rclcpp)

target_include_directories(ctrlboard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(ctrlboard_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/istrorsx_hw/istrobtx>
  ${OpenCV_INCLUDE_DIRS}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_compile_features(ctrlboard_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(ctrlboard_node PRIVATE
  rclcpp::rclcpp
  log4cxx
  ${OpenCV_LIBRARIES}
  istrobtx::istrobtx
  "${cpp_typesupport_target}"
)

#ament_export_dependencies(rosidl_default_runtime)

install(TARGETS ctrlboard_node
  DESTINATION lib/${PROJECT_NAME})

# gps_node: separate executable (see 05_migration_guide.md) -- own source list
# and own extra linkage (libgps, GeographicLib) that ctrlboard_node doesn't need.
set(GPS_NODE_SOURCES
  src/gps_node.cpp
  src/istrobtx/gpsdev.cpp
  src/istrobtx/istro_main_gps.cpp
)
add_executable(gps_node ${GPS_NODE_SOURCES})

target_include_directories(gps_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(gps_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/istrorsx_hw/istrobtx>
  ${OpenCV_INCLUDE_DIRS}
  ${GPS_INCLUDE_DIRS}
)

target_compile_features(gps_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(gps_node PRIVATE
  rclcpp::rclcpp
  log4cxx
  ${OpenCV_LIBRARIES}
  ${GPS_LIBRARIES}
  istrobtx::istrobtx
  "${cpp_typesupport_target}"
)

install(TARGETS gps_node
  DESTINATION lib/${PROJECT_NAME})

# lidar_node: separate executable (see 05_migration_guide.md) -- own source
# list and own extra linkage (rplidar_sdk) that ctrlboard_node/gps_node don't
# need. lidar.cpp only (no ctrlboard.cpp, no gpsdev.cpp/geocalc.cpp) --
# dmap.cpp (DegreeMap) moved into the shared istrobtx package, since
# istrorsx_core's vision_node needs it too now (see 01_architecture.md).
set(LIDAR_NODE_SOURCES
  src/lidar_node.cpp
  src/istrobtx/istro_main_lidar.cpp
  src/istrobtx/lidar.cpp
)
add_executable(lidar_node ${LIDAR_NODE_SOURCES})

target_include_directories(lidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(lidar_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/istrorsx_hw/istrobtx>
  ${OpenCV_INCLUDE_DIRS}
  ${RPLIDAR_SDK_INCLUDE_DIR}
)

target_compile_features(lidar_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(lidar_node PRIVATE
  rclcpp::rclcpp
  log4cxx
  ${OpenCV_LIBRARIES}
  ${RPLIDAR_SDK_LIBRARY}
  pthread
  istrobtx::istrobtx
  "${cpp_typesupport_target}"
)

install(TARGETS lidar_node
  DESTINATION lib/${PROJECT_NAME})

# camera_node: separate executable (see 05_migration_guide.md) -- own source
# list and own extra linkage (realsense2) that the other three don't need.
# camera.cpp only (no camera_opencv.cpp -- not migrated, see 03_progress.md).
# The same binary is launched twice (front/rear), selected at runtime via the
# -camdev CLI flag (see Config::cameraDeviceId) -- there is no separate
# CAMERA2_NODE_SOURCES/executable.
set(CAMERA_NODE_SOURCES
  src/camera_node.cpp
  src/istrobtx/camera.cpp
  src/istrobtx/istro_main_camera.cpp
)
add_executable(camera_node ${CAMERA_NODE_SOURCES})

target_include_directories(camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(camera_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/istrorsx_hw/istrobtx>
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
)

target_compile_features(camera_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(camera_node PRIVATE
  rclcpp::rclcpp
  log4cxx
  ${OpenCV_LIBRARIES}
  ${realsense2_LIBRARY}
  istrobtx::istrobtx
  "${cpp_typesupport_target}"
)

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME})

# ups_node: Waveshare UPS Power Module (C) battery telemetry (INA219 over
# I2C). No legacy equivalent -- own minimal source list, no OpenCV/libgps/
# rplidar_sdk/realsense2 linkage, and no Config/Threads/DataSet scaffold
# (see ups_node.hpp's own comment).
set(UPS_NODE_SOURCES
  src/ups_node.cpp
  src/istrobtx/ina219.cpp
)
add_executable(ups_node ${UPS_NODE_SOURCES})

target_include_directories(ups_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(ups_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/istrorsx_hw/istrobtx>
)

target_compile_features(ups_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(ups_node PRIVATE
  rclcpp::rclcpp
  log4cxx
  istrobtx::istrobtx
  "${cpp_typesupport_target}"
)

install(TARGETS ups_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
