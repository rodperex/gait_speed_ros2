cmake_minimum_required(VERSION 3.8)
project(gait_speed_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(perception_system_interfaces REQUIRED)
find_package(perception_system REQUIRED)
find_package(motion_bt_nodes REQUIRED)
find_package(perception_bt_nodes REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  ament_index_cpp
  rclcpp_cascade_lifecycle
  behaviortree_cpp_v3
  std_msgs
  perception_system_interfaces
  perception_system
  motion_bt_nodes
  perception_bt_nodes
  tf2_geometry_msgs
  tf2
  tf2_ros
)

include_directories(include)

add_library(check_end_test_bt_node SHARED src/gait_speed_ros2/bt_nodes/CheckEndTest.cpp)
add_library(start_test_bt_node SHARED src/gait_speed_ros2/bt_nodes/StartTest.cpp)
add_library(end_test_bt_node SHARED src/gait_speed_ros2/bt_nodes/EndTest.cpp)


list(APPEND plugin_libs
  start_test_bt_node
  check_end_test_bt_node
  end_test_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


add_library(${PROJECT_NAME} SHARED
  src/gait_speed_ros2/orchestrators/example.cpp
  src/gait_speed_ros2/orchestrators/example2.cpp
  src/gait_speed_ros2/orchestrators/gait_speed.cpp
  src/gait_speed_ros2/behaviors/behavior_runner.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(example_nav src/main_example_nav.cpp)
ament_target_dependencies(example_nav ${dependencies})
target_link_libraries(example_nav ${PROJECT_NAME})

add_executable(example_repeat src/main_example_repeat.cpp)
ament_target_dependencies(example_repeat ${dependencies})
target_link_libraries(example_repeat ${PROJECT_NAME})

add_executable(gait_speed_dist src/main_gait_speed_dist.cpp)
ament_target_dependencies(gait_speed_dist ${dependencies})
target_link_libraries(gait_speed_dist ${PROJECT_NAME})

install(TARGETS
  ${plugin_libs}
  ${PROJECT_NAME}
  example_nav
  example_repeat
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