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

#include "gait_speed_ros2/orchestrators/gait_speed_simple.hpp"
#include "gait_speed_ros2/orchestrators/gait_speed_states.hpp"
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

  // orchestrator
  auto gait_speed_node = std::make_shared<gait_speed::GaitSpeedSimple>(blackboard);
  gait_speed_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  gait_speed_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // behaviors
  std::vector<std::string> plugins;
  
  plugins = {
    "is_detected_bt_node",
    "identify_bt_node",
    "navigate_to_bt_node"
  };

  auto find_person_node = std::make_shared<gait_speed::BehaviorRunner>(
    blackboard,
    "find_patient",
    "/bt_xml/find_person_simple.xml",
    plugins
  );
  
  plugins = {
    "start_test_bt_node",
    "spin_bt_node",
    "is_in_view_bt_node",
    "identify_bt_node",
    "check_end_test_bt_node",
    "end_test_bt_node",
    "has_person_started_bt_node",
    "navigate_to_bt_node",
    "get_detection_from_tf_bt_node",
    "is_in_front_bt_node",
    "is_detected_bt_node"};

  auto measure_node = std::make_shared<gait_speed::BehaviorRunner>(
    blackboard,
    "measure_gait_speed",
    "/bt_xml/measure_gait_speed_no_hri.xml",
    plugins
  );

  exec.add_node(gait_speed_node->get_node_base_interface()); // Orchestrator
  // exec.add_node(node->get_node_base_interface()); // BT node
  exec.add_node(find_person_node->get_node_base_interface()); // BehaviorRunner: find_person
  exec.add_node(measure_node->get_node_base_interface()); // BehaviorRunner: measure_gait_speed

  // Leave behavior runners ready to be activated
  find_person_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  measure_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  RCLCPP_INFO(node->get_logger(), "gait_speed_node state: %s", gait_speed_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "find_person_node state: %s", find_person_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "measure_node state: %s", measure_node->get_current_state().label().c_str());
  
  while (rclcpp::ok()) {
    exec.spin_some();
    
    if (gait_speed_node->get_state() == gait_speed::State::STOP) {
      RCLCPP_INFO(node->get_logger(), "Orchestrator stopped. Exiting...");
      
      gait_speed_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

      break;
    }
  }

  RCLCPP_INFO(node->get_logger(), "gait_speed_node state: %s", gait_speed_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "find_person_node state: %s", find_person_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "measure_node state: %s", measure_node->get_current_state().label().c_str());

  rclcpp::shutdown();

  return 0;
}
