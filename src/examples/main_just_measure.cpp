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

#include "gait_speed_ros2/behaviors/behavior_runner.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  
  auto node = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("bt_node");
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);


  std::vector<std::string> plugins;
  
  plugins = {
    "is_detected_bt_node",
    "is_in_front_bt_node",
    "start_test_bt_node",
    "has_person_started_bt_node",
    "navigate_to_bt_node",
    "spin_bt_node",
    "identify_bt_node",
    "speak_bt_node",
    "check_end_test_bt_node",
    "end_test_bt_node"};

  auto measure = std::make_shared<gait_speed::BehaviorRunner>(
    blackboard,
    "measure",
    "/bt_xml/just_measure_gait_speed.xml",
    plugins
  );
 
  exec.add_node(measure->get_node_base_interface());
  
  RCLCPP_INFO(node->get_logger(), "Behaviors added to executor");

  measure->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  measure->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  RCLCPP_INFO(node->get_logger(), "Measure state: %s", measure->get_current_state().label().c_str());
  
  RCLCPP_INFO(node->get_logger(), "Behaviors configured");
  
  while (rclcpp::ok()) {
    exec.spin_some();
    if ((measure->get_bt_status() == BT::NodeStatus::SUCCESS) || measure->get_bt_status() == BT::NodeStatus::FAILURE) {
      RCLCPP_INFO(node->get_logger(), "Orchestrator stopped. Exiting...");
      measure->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      break;
    }
  }

  RCLCPP_INFO(node->get_logger(), "Measure state: %s", measure->get_current_state().label().c_str());
  
  rclcpp::shutdown();

  return 0;
}
