cmake_minimum_required(VERSION 3.8)  # Changed from 3.20 to match ROS2 humble/foxy
project(ultrasonic_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Creates variables like rclcpp_LIBRARIES and std_msgs_INCLUDE_DIRS
#
find_package(ament_cmake REQUIRED)  # ROS2 build system
find_package(rclcpp REQUIRED)       # ROS2 C++ client library
find_package(std_msgs REQUIRED)     # ROS2 standard msg types

# Adds include/ directory to compiler's search path for ALL targets
#   This is why #include "ultrasonic_pkg/lidaar/RPLidar.hpp" works
#
include_directories(
  include
)

# Creates variable LIB_SOURCE_FILES containing (wait for it) a list of source files
#   used to organize source files for the library
#
set(LIB_SOURCE_FILES
  src/lidar/RPLidar.cpp
  src/lidar/SerialCommunication.cpp
  src/data_wrappers/ExpressScanPacket.cpp
  src/data_wrappers/RequestPacket.cpp
)

# Loop through each file in LIB_SOURCE_FILES
foreach(src_file ${LIB_SOURCE_FILES})
  # Check if the source file exists
  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${src_file})
    message(WARNING "Source file not found: ${src_file}")
  else()
    message(STATUS "Found source file: ${src_file}")
  endif()
endforeach()

# Create library from source files
add_library(${PROJECT_NAME}_lib ${LIB_SOURCE_FILES})

# Add include paths to this specific target
#   PUBLIC indicates that both the library and anything linking to it get these includes
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  include
)

# Add include directories from dependencies, links libraries, and propagates compiler flags
#
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# Main node executable
add_executable(ultrasonic_node src/ultrasonic_node.cpp)

# PRIVATE includes for only this executable, not propagated
target_include_directories(ultrasonic_node PRIVATE include)

# Link "ultrasonic_pkg_lib" to executable
#   Can now use functions/classes from library
target_link_libraries(ultrasonic_node PUBLIC
  ${PROJECT_NAME}_lib
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

install(TARGETS                   # Copies built files to install directory
  ultrasonic_node                 # Executable goes to lib/ultrasonic_pkg/
  DESTINATION lib/${PROJECT_NAME} # Executables go here for ROS2
)

install(TARGETS
  ${PROJECT_NAME}_lib             # Library also installed
  ARCHIVE DESTINATION lib         # Static libraries (.a)
  LIBRARY DESTINATION lib         # Shared libraries (.so)
)

# Installs all header files from include/ to install/include/
#   Allows other packages to use headers
install(DIRECTORY include/
  DESTINATION include/
)

# Makes package discoverable by other ROS2 packages
#   Other packages using fid_package(ultrasonic_pkg) will get:
ament_export_include_directories(include)   # Include directories
ament_export_libraries(${PROJECT_NAME}_lib) # Library to link against
ament_export_dependencies(rclcpp std_msgs)  # Dependencies transitively

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()