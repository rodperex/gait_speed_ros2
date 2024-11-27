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
    "is_pointing_bt_node",
    "spin_bt_node",
    "identify_person_bt_node",
    "navigate_to_bt_node"
  };

  auto find = std::make_shared<gait_speed::BehaviorRunner>(
    blackboard,
    "find",
    "/bt_xml/find_person.xml",
    plugins
  );
 
  exec.add_node(find->get_node_base_interface());
  
  RCLCPP_INFO(node->get_logger(), "Behaviors added to executor");

  find->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  find->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  RCLCPP_INFO(node->get_logger(), "Find state: %s", find->get_current_state().label().c_str());
  
  RCLCPP_INFO(node->get_logger(), "Behaviors configured");
  
  while (rclcpp::ok()) {
    exec.spin_some();
    if (find->get_bt_status() != BT::NodeStatus::RUNNING) {
      RCLCPP_INFO(node->get_logger(), "Orchestrator stopped. Exiting...");
      find->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      break;
    }
  }

  RCLCPP_INFO(node->get_logger(), "Find state: %s", find->get_current_state().label().c_str());
  
  rclcpp::shutdown();

  return 0;
}
