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
find_package(std_srvs REQUIRED)

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
  std_srvs
)

include_directories(include)

add_library(check_end_test_bt_node SHARED src/gait_speed_ros2/bt_nodes/CheckEndTest.cpp)
add_library(start_test_bt_node SHARED src/gait_speed_ros2/bt_nodes/StartTest.cpp)
add_library(start_test_mocap_bt_node SHARED src/gait_speed_ros2/bt_nodes/StartTestMocap.cpp)
add_library(has_person_started_bt_node SHARED src/gait_speed_ros2/bt_nodes/HasPersonStarted.cpp)
add_library(end_test_bt_node SHARED src/gait_speed_ros2/bt_nodes/EndTest.cpp)


list(APPEND plugin_libs
  start_test_bt_node
  start_test_mocap_bt_node
  has_person_started_bt_node
  check_end_test_bt_node
  end_test_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


add_library(${PROJECT_NAME} SHARED
  src/gait_speed_ros2/orchestrators/examples/example_nav.cpp
  src/gait_speed_ros2/orchestrators/examples/example_repeat.cpp
  src/gait_speed_ros2/orchestrators/gait_speed.cpp
  src/gait_speed_ros2/orchestrators/gait_speed_simple.cpp
  src/gait_speed_ros2/behaviors/behavior_runner.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(example_nav src/examples/main_example_nav.cpp)
ament_target_dependencies(example_nav ${dependencies})
target_link_libraries(example_nav ${PROJECT_NAME})

add_executable(example_repeat src/examples/main_example_repeat.cpp)
ament_target_dependencies(example_repeat ${dependencies})
target_link_libraries(example_repeat ${PROJECT_NAME})

add_executable(just_explain src/examples/main_just_explain.cpp)
ament_target_dependencies(just_explain ${dependencies})
target_link_libraries(just_explain ${PROJECT_NAME})

add_executable(explain_and_focus src/examples/main_explain_and_focus.cpp)
ament_target_dependencies(explain_and_focus ${dependencies})
target_link_libraries(explain_and_focus ${PROJECT_NAME})

add_executable(just_find src/examples/main_just_find.cpp)
ament_target_dependencies(just_find ${dependencies})
target_link_libraries(just_find ${PROJECT_NAME})

add_executable(just_measure src/examples/main_just_measure.cpp)
ament_target_dependencies(just_measure ${dependencies})
target_link_libraries(just_measure ${PROJECT_NAME})

add_executable(gait_speed src/main_gait_speed.cpp)
ament_target_dependencies(gait_speed ${dependencies})
target_link_libraries(gait_speed ${PROJECT_NAME})

add_executable(sim_gait_speed src/sim_main_gait_speed.cpp)
ament_target_dependencies(sim_gait_speed ${dependencies})
target_link_libraries(sim_gait_speed ${PROJECT_NAME})

add_executable(gait_speed_simple src/main_gait_speed_simple.cpp)
ament_target_dependencies(gait_speed_simple ${dependencies})
target_link_libraries(gait_speed_simple ${PROJECT_NAME})

add_executable(mocap_gait_speed_simple src/main_gait_speed_simple_mocap.cpp)
ament_target_dependencies(mocap_gait_speed_simple ${dependencies})
target_link_libraries(mocap_gait_speed_simple ${PROJECT_NAME})

add_executable(gait_speed_mocap src/main_gait_speed_mocap.cpp)
ament_target_dependencies(gait_speed_mocap ${dependencies})
target_link_libraries(gait_speed_mocap ${PROJECT_NAME})

install(TARGETS
  ${plugin_libs}
  ${PROJECT_NAME}
  example_nav
  example_repeat
  just_explain
  explain_and_focus
  just_find
  just_measure
  gait_speed
  sim_gait_speed
  gait_speed_simple
  gait_speed_mocap
  mocap_gait_speed_simple
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


ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

# foreach(bt_plugin ${plugin_libs})
#   ament_export_libraries(${bt_plugin})
# endforeach()
  
# ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()