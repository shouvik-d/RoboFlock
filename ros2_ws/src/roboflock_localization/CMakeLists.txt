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

target_include_directories(${PROJECT_NAME}_lib PUBLIC
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
	"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME}_lib PUBLIC
	${robot_localization_LIBRARIES}
)

#install(DIRECTORY include/${PROJECT_NAME}
#	DESTINATION include
#)

install(TARGETS ${PROJECT_NAME}_lib
	EXPORT ${PROJECT_NAME}_lib_targets
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	#INCLUDES DESTINATION include/${PROJECT_NAME}
)

#ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(${PROJECT_NAME}_lib_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(robot_localization)

install(DIRECTORY
	launch
	params
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()
