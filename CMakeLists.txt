cmake_minimum_required(VERSION 3.8)
project(gait_speed_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(perception_system REQUIRED)
find_package(perception_system_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  rclcpp_cascade_lifecycle
  behaviortree_cpp_v3
  std_msgs
  perception_system
  perception_system_interfaces
  tf2_geometry_msgs
  tf2
  tf2_ros
)

include_directories(include)

add_library(is_my_person_bt_node SHARED src/gait_speed_ros2/bt_nodes/isMyPerson.cpp)
add_library(distance_reached_bt_node SHARED src/gait_speed_ros2/bt_nodes/DistanceReached.cpp)
add_library(target_reached_bt_node SHARED src/gait_speed_ros2/bt_nodes/TargetReached.cpp)
add_library(start_test_bt_node SHARED src/gait_speed_ros2/bt_nodes/StartTest.cpp)
add_library(end_test_bt_node SHARED src/gait_speed_ros2/bt_nodes/EndTest.cpp)


list(APPEND plugin_libs
  is_my_person_bt_node
  distance_reached_bt_node
  target_reached_bt_node
  start_test_bt_node
  end_test_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


add_library(${PROJECT_NAME} SHARED
  src/gait_speed_ros2/orchestrators/gait_speed.cpp
  src/gait_speed_ros2/behaviors/behavior_runner.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(gait_speed_dist src/main_gait_speed_dist.cpp)
ament_target_dependencies(gait_speed_dist ${dependencies})
target_link_libraries(gait_speed_dist ${PROJECT_NAME})

install(TARGETS
  ${plugin_libs}
  ${PROJECT_NAME}
  gait_speed_dist
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config bt_xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
# ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()