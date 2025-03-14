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
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  

  std::vector<std::string> plugins;
  
  plugins = {
    "speak_bt_node",
    "listen_bt_node",
    "dialog_confirmation_bt_node",
    "activate_attention_bt_node",
    "deactivate_attention_bt_node",
    "is_detected_bt_node",
    "identify_bt_node"
  };

  auto explain_and_focus = std::make_shared<gait_speed::BehaviorRunner>(
    blackboard,
    "gait_speed_node",
    "/bt_xml/explain_and_focus.xml", 
    plugins
  );
 
  exec.add_node(explain_and_focus->get_node_base_interface());
  
  RCLCPP_INFO(node->get_logger(), "Behaviors added to executor");

  explain_and_focus->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  explain_and_focus->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  RCLCPP_INFO(node->get_logger(), "Explain state: %s", explain_and_focus->get_current_state().label().c_str());
  
  RCLCPP_INFO(node->get_logger(), "Behaviors configured");
 
  while (rclcpp::ok()) {
    exec.spin_some();
    if (explain_and_focus->get_bt_status() == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Explain and focus stopped: SUCCESS");
      explain_and_focus->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      break;
    }
    if (explain_and_focus->get_bt_status() == BT::NodeStatus::FAILURE) {
      RCLCPP_INFO(node->get_logger(), "Explain and focus stopped: FAILURE");
      explain_and_focus->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      break;
    }
  }

  RCLCPP_INFO(node->get_logger(), "Explain and focus state: %s", explain_and_focus->get_current_state().label().c_str());
  
  rclcpp::shutdown();

  return 0;
}
