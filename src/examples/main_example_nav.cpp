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

#include "gait_speed_ros2/orchestrators/examples/example_nav.hpp"

#include "gait_speed_ros2/behaviors/behavior_runner.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);   // set number of threads to whatever needed

  rclcpp::executors::SingleThreadedExecutor exec;
  // cambiar a eventExecutor
  
  auto node = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("bt_node");
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  // orchestrator
  auto orch = std::make_shared<example::ExampleNav>(blackboard);

  // behaviors
  std::vector<std::string> plugins;
  
  plugins = {
    "speak_bt_node",
    "listen_bt_node"
  };

  auto listen_and_repeat_node = std::make_shared<gait_speed::BehaviorRunner>(
    blackboard,
    "listen_and_repeat",
    "/bt_xml/examples/listen_and_repeat.xml",
    plugins
  );

  plugins = {
    "navigate_through_bt_node",
    "navigate_to_bt_node"
  };

  auto navigation_node = std::make_shared<gait_speed::BehaviorRunner>(
    blackboard,
    "navigation",
    "/bt_xml/examples/nav_through.xml",
    plugins
  );

  exec.add_node(listen_and_repeat_node->get_node_base_interface());
  exec.add_node(navigation_node->get_node_base_interface());
  exec.add_node(orch->get_node_base_interface());

  RCLCPP_INFO(node->get_logger(), "Behaviors added to executor");

  orch->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  orch->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  listen_and_repeat_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  navigation_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  
  RCLCPP_INFO(node->get_logger(), "Orchestrator state: %s", orch->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "Listen and repeat state: %s", listen_and_repeat_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "Navigation state: %s", navigation_node->get_current_state().label().c_str());
  
  
  RCLCPP_INFO(node->get_logger(), "Behaviors configured");
  
  while (rclcpp::ok()) {
    exec.spin_some();
    
    if (orch->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_INFO(node->get_logger(), "Orchestrator inactive. Exiting...");
      break;
    }

  }

  RCLCPP_INFO(node->get_logger(), "Orchestrator state: %s", orch->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "Listen and repeat state: %s", listen_and_repeat_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "Navigation state: %s", navigation_node->get_current_state().label().c_str());
  
  rclcpp::shutdown();

  return 0;
}
