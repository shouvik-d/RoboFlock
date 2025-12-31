cmake_minimum_required(VERSION 3.20)
project(ultrasonic_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(lvmax_publisher src/lvmax_publisher.cpp)

target_link_libraries (lvmax_publisher PUBLIC 
  rclcpp::rclcpp 
  ${std_msgs_TARGETS}
)

install(TARGETS lvmax_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()