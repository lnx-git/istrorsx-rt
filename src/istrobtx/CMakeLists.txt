cmake_minimum_required(VERSION 3.10)
project(istrobtx)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(log4cxx REQUIRED)
find_package(OpenCV REQUIRED)

# GeographicLib (for GeoCalc) ships no CMake config package -- discovered via
# pkg-config instead, like istrorsx_hw already does for libgps. GeoCalc moved
# here (from istrorsx_hw) since it's needed identically by istrorsx_hw's
# gps_node AND istrorsx_core's navigation_node. See doc/ai/01_architecture.md.
find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

add_library(istrobtx STATIC
  src/camera_defs.cpp
  src/config.cpp
  src/ctrlboard_defs.cpp
  src/dataset.cpp
  src/dmap.cpp
  src/geocalc.cpp
  src/lidar_defs.cpp
  src/logger.cpp
  src/mtime.cpp
  src/system.cpp
  src/threads.cpp
)

# PUBLIC: dataset.h/logger.h/geocalc.h use cv::Mat/log4cxx/GeographicLib types
# directly in their public interface, so any consumer including them needs
# these on its own include path too, not just this library's own .cpp files.
target_include_directories(istrobtx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${GEOGRAPHICLIB_INCLUDE_DIRS}
)

target_compile_features(istrobtx PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(istrobtx PUBLIC
  log4cxx
  ${OpenCV_LIBRARIES}
  ${GEOGRAPHICLIB_LIBRARIES}
  pthread
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS istrobtx
  EXPORT istrobtx_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(istrobtx_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(log4cxx OpenCV)

ament_package()
