// Copyright 2024 Rodrigo Pérez-Rodríguez
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gait_speed_ros2/orchestrators/gait_speed_dist.hpp"

#include "gait_speed_ros2/behaviors/behavior_runner.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);   // set number of threads to whatever needed

  auto node = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("bt_node");


  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  // orchestrator
  auto gait_speed_node = std::make_shared<gait_speed::GaitSpeedDist>(blackboard);

  // behaviors
  std::vector<std::string> plugins;
  
  plugins = {
    "follow_person_bt_node",
    "look_at_bt_node",
    "filter_entity_bt_node",
    "pan_bt_node",
    "start_test_bt_node",
    "is_detected_bt_node",
    "distance_reached_bt_node",
    "is_my_person_bt_node",
    "end_test_bt_node"};
  auto measure_gait_speed_node = std::make_shared<gait_speed::BehaviorRunner>(
    blackboard,
    "measure_gait_speed",
    "/bt_xml/measure_gait_speed_dist.xml",
    plugins
    );

  // plugins = {
  //   "is_waving_bt_node",
  //   "pan_bt_node",
  //   "spin_bt_node",
  //   "move_to_bt_node",
  //   "is_detected_bt_node",
  //   "filter_entity_bt_node",
  //   "look_at_bt_node"
  //   };
  // auto find_person_node = std::make_shared<gait_speed::BehaviorRunner>(
  //   blackboard,
  //   "find_person",
  //   "/bt_xml/find_person.xml",
  //   plugins
  //   );

  exec.add_node(gait_speed_node->get_node_base_interface());
  exec.add_node(measure_gait_speed_node->get_node_base_interface());
  // exec.add_node(find_person_node->get_node_base_interface());

  gait_speed_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  measure_gait_speed_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  // find_person_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
