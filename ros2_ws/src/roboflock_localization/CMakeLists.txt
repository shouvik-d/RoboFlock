cmake_minimum_required(VERSION 3.8)
project(roboflock_localization)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_localization REQUIRED)


add_library(${PROJECT_NAME}_lib
  src/rf_localization_node.cpp
)

target_link_libraries(${PROJECT_NAME}_lib PUBLIC
	${robot_localization_LIBRARIES}
)

ament_export_dependencies(robot_localization)

install(DIRECTORY
	launch
	params
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()
