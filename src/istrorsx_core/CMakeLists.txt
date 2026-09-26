cmake_minimum_required(VERSION 3.10)
project(istrorsx_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(istrorsx_hw REQUIRED)
find_package(istrobtx REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# libxml2 (for navmap.cpp's navmap_load(), ISTRO_LIBXML2 in system.h) ships
# no CMake config package -- discovered via pkg-config instead, like
# istrorsx_hw already does for libgps. Only navigation_node needs this.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)

# ZBar (for qrscan.cpp's QRScanner, ISTRO_QRSCANNER in system.h) ships no
# CMake config package either -- same pkg-config route. Only vision_node
# needs this.
pkg_check_modules(ZBAR REQUIRED zbar)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConfigUpdate.msg"
  "msg/DriveCommand.msg"
  "msg/NavigationData.msg"
  "msg/NavigationRoute.msg"
  "msg/NavMapNode.msg"
  "msg/NavMapSegment.msg"
  "msg/VisionData.msg"
  "msg/VisionDebugData.msg"
  "msg/VisionControl.msg"
  "msg/PlannerData.msg"
  "msg/PlannerDebugData.msg"
  "msg/NavigationPointSet.msg"
  "msg/SaveData.msg"
  "msg/SaveEvent.msg"
  "srv/GetWMGridSnapshot.srv"
  "srv/NavMapExport.srv"
)

set(DRIVE_NODE_SOURCES
  src/drive_node.cpp
  src/istrobtx/istro_main_drive.cpp
)
add_executable(drive_node ${DRIVE_NODE_SOURCES})

target_include_directories(drive_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(drive_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_target_dependencies(drive_node rclcpp sensor_msgs istrorsx_hw)

# Plain (non-keyword) signature -- must match ament_target_dependencies()
# above, which always uses the plain form; mixing keyword (PRIVATE/PUBLIC)
# and plain target_link_libraries() calls on the same target is a CMake error.
target_link_libraries(drive_node
  istrobtx::istrobtx
  "${cpp_typesupport_target}"
)

install(TARGETS drive_node
  DESTINATION lib/${PROJECT_NAME})

# keyboard_node: terminal-based replacement for legacy loop_waitKey()'s
# cv::waitKey()/X11 key detection (see doc/ai/01_architecture.md). Publishes
# sensor_msgs/msg/Joy using drive_node.hpp's own JOY_AXIS_*/JOY_BUTTON_*
# constants, so it #includes drive_node.hpp (compile-time constants only, no
# runtime coupling) and needs the same rclcpp/sensor_msgs/istrorsx_hw deps.
# Also links istrobtx for mtime's timeBegin()/timeEnd(), so publishTick() is
# measured like every other publishXxx() in the workspace -- the one piece of
# runtime coupling this node has.
# Needs std::thread (background terminal reader) -> pthread.
find_package(Threads REQUIRED)

add_executable(keyboard_node src/keyboard_node.cpp)

target_include_directories(keyboard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(keyboard_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(keyboard_node rclcpp sensor_msgs istrorsx_hw)

# Plain (non-keyword) signature -- must match ament_target_dependencies()
# above, which always uses the plain form; mixing keyword (PRIVATE/PUBLIC)
# and plain target_link_libraries() calls on the same target is a CMake error.
target_link_libraries(keyboard_node
  "${cpp_typesupport_target}"
  Threads::Threads
  istrobtx::istrobtx
)

install(TARGETS keyboard_node
  DESTINATION lib/${PROJECT_NAME})

# navigation_node: separate executable -- own source list (navig.cpp/
# navig_data.cpp/navmap.cpp/navmap_data.cpp, port of gps_thread's navigation-
# point/route-planning extension) and own extra linkage (libxml2) that
# drive_node doesn't need. See doc/ai/01_architecture.md § navigation_node.
set(NAVIGATION_NODE_SOURCES
  src/navigation_node.cpp
  src/istrobtx/istro_main_navigation.cpp
  src/istrobtx/navig.cpp
  src/istrobtx/navig_data.cpp
  src/istrobtx/navmap.cpp
  src/istrobtx/navmap_data.cpp
)
add_executable(navigation_node ${NAVIGATION_NODE_SOURCES})

target_include_directories(navigation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(navigation_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/istrorsx_core/istrobtx>
  ${LIBXML2_INCLUDE_DIRS}
)

target_compile_features(navigation_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(navigation_node rclcpp istrorsx_hw)

# Plain (non-keyword) signature -- must match ament_target_dependencies()
# above, which always uses the plain form; mixing keyword (PRIVATE/PUBLIC)
# and plain target_link_libraries() calls on the same target is a CMake error.
target_link_libraries(navigation_node
  istrobtx::istrobtx
  ${LIBXML2_LIBRARIES}
  "${cpp_typesupport_target}"
)

install(TARGETS navigation_node
  DESTINATION lib/${PROJECT_NAME})

# vision_node: separate executable -- own source list (vision.cpp/
# vision_depth.cpp/visionn.cpp/qrscan.cpp/sample.cpp, port of vision_thread)
# and own extra linkage (zbar) that drive_node/navigation_node don't need.
# See doc/ai/01_architecture.md § vision_node.
set(VISION_NODE_SOURCES
  src/vision_node.cpp
  src/istrobtx/istro_main_vision.cpp
  src/istrobtx/vision.cpp
  src/istrobtx/vision_depth.cpp
  src/istrobtx/visionn.cpp
  src/istrobtx/qrscan.cpp
  src/istrobtx/sample.cpp
)
add_executable(vision_node ${VISION_NODE_SOURCES})

target_include_directories(vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(vision_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/istrorsx_core/istrobtx>
  ${ZBAR_INCLUDE_DIRS}
)

target_compile_features(vision_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(vision_node rclcpp istrorsx_hw)

# Plain (non-keyword) signature -- must match ament_target_dependencies()
# above, which always uses the plain form; mixing keyword (PRIVATE/PUBLIC)
# and plain target_link_libraries() calls on the same target is a CMake error.
target_link_libraries(vision_node
  istrobtx::istrobtx
  ${ZBAR_LIBRARIES}
  "${cpp_typesupport_target}"
)

install(TARGETS vision_node
  DESTINATION lib/${PROJECT_NAME})

# planner_node: separate executable -- own source list (wmodel.cpp, port of
# process_thread's global WorldModel; navig.cpp/navig_data.cpp/navmap.cpp/
# navmap_data.cpp linked in too, not because planner_node calls them itself,
# but because wmodel.cpp's drawGridFull() -- never called by planner_node,
# exclusive to the future save_node -- still references their symbols
# (navigation_point_cnt()/navigationPointXY[]/navigationPoint[]) at link
# time, since wmodel.cpp compiles as one translation unit. See
# doc/ai/01_architecture.md § planner_node. Needs libxml2 (navmap.cpp) same
# as navigation_node.
set(PLANNER_NODE_SOURCES
  src/planner_node.cpp
  src/istrobtx/istro_main_planner.cpp
  src/istrobtx/wmodel.cpp
  src/istrobtx/navig.cpp
  src/istrobtx/navig_data.cpp
  src/istrobtx/navmap.cpp
  src/istrobtx/navmap_data.cpp
)
add_executable(planner_node ${PLANNER_NODE_SOURCES})

target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(planner_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/istrorsx_core/istrobtx>
  ${LIBXML2_INCLUDE_DIRS}
)

target_compile_features(planner_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(planner_node rclcpp istrorsx_hw)

# Plain (non-keyword) signature -- must match ament_target_dependencies()
# above, which always uses the plain form; mixing keyword (PRIVATE/PUBLIC)
# and plain target_link_libraries() calls on the same target is a CMake error.
target_link_libraries(planner_node
  istrobtx::istrobtx
  ${LIBXML2_LIBRARIES}
  "${cpp_typesupport_target}"
)

install(TARGETS planner_node
  DESTINATION lib/${PROJECT_NAME})

# save_node: separate executable -- own source list. vision.cpp (Vision::
# drawOutput() only, no QRScanner/VisioNN/VisionDepth needed -- see
# doc/ai/01_architecture.md § save_node) + sample.cpp (SamplePixels, needed
# for initVision()'s calibration-sample loading -- also where vision.cpp's
# own convert()/convertInv() free functions live, so it's a real link-time
# dependency, not just an initVision() one); wmodel.cpp (own WorldModel
# instance, drawGrid()/drawGridFull()); navig.cpp/navig_data.cpp/navmap.cpp/
# navmap_data.cpp both because save_node calls navmap_draw()/
# navmap_export_kml() itself (unlike planner_node, where these were only a
# link-time-only dependency of wmodel.cpp's drawGridFull()) and because
# wmodel.cpp needs them for the same reason it does in planner_node. Needs
# libxml2 (navmap.cpp) same as navigation_node/planner_node. No zbar (no
# QRScanner instance).
set(SAVE_NODE_SOURCES
  src/save_node.cpp
  src/istrobtx/istro_main_save.cpp
  src/istrobtx/vision.cpp
  src/istrobtx/sample.cpp
  src/istrobtx/wmodel.cpp
  src/istrobtx/navig.cpp
  src/istrobtx/navig_data.cpp
  src/istrobtx/navmap.cpp
  src/istrobtx/navmap_data.cpp
)
add_executable(save_node ${SAVE_NODE_SOURCES})

target_include_directories(save_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(save_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/istrorsx_core/istrobtx>
  ${LIBXML2_INCLUDE_DIRS}
)

target_compile_features(save_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(save_node rclcpp istrorsx_hw)

# Plain (non-keyword) signature -- must match ament_target_dependencies()
# above, which always uses the plain form; mixing keyword (PRIVATE/PUBLIC)
# and plain target_link_libraries() calls on the same target is a CMake error.
target_link_libraries(save_node
  istrobtx::istrobtx
  ${LIBXML2_LIBRARIES}
  "${cpp_typesupport_target}"
)

install(TARGETS save_node
  DESTINATION lib/${PROJECT_NAME})

# telemetry_node: port of legacy script/logp.cpp -- unlike every other node
# here, needs no istro_main_<subsystem>() scaffold (Config/Threads/DataSet)
# and no istrorsx_hw messages -- legacy logp.cpp was itself a fully
# standalone program with no shared code, see 01_architecture.md's
# telemetry_node section. Still links istrobtx (LOG_DEFINE/LOGM_* only).
add_executable(telemetry_node src/telemetry_node.cpp)

target_include_directories(telemetry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(telemetry_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(telemetry_node rclcpp)

# Plain (non-keyword) signature -- must match ament_target_dependencies()
# above, which always uses the plain form; mixing keyword (PRIVATE/PUBLIC)
# and plain target_link_libraries() calls on the same target is a CMake error.
target_link_libraries(telemetry_node
  istrobtx::istrobtx
  "${cpp_typesupport_target}"
  Threads::Threads
)

install(TARGETS telemetry_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
