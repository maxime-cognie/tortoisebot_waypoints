cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2)
find_package(rclcpp_action)

set(dependencies
	rclcpp
	geometry_msgs
	nav_msgs
	tf2
	rclcpp_action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES std_msgs geometry_msgs
)


## ADD LIBRARY
include_directories(include)
add_library(tortoisebot_waypoints_lib src/tortoisebot_action_server.cpp include/tortoisebot_waypoints/tortoisebot_action_server.h)
ament_target_dependencies(tortoisebot_waypoints_lib ${dependencies})
rosidl_target_interfaces(tortoisebot_waypoints_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")


ament_export_targets(tortoisebot_waypoints_lib HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/tortoisebot_waypoints
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_find_gtest()
  add_subdirectory(test)
endif()

## ADD EXECUTABLE
add_executable(main src/main.cpp)
target_link_libraries(main PUBLIC tortoisebot_waypoints_lib)

install(TARGETS tortoisebot_waypoints_lib
  DESTINATION lib/tortoisebot_waypoints_lib)

install(TARGETS 
  main
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS tortoisebot_waypoints_lib
  EXPORT tortoisebot_waypoints_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
