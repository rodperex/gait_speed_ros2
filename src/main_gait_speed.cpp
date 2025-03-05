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

#include "gait_speed_ros2/orchestrators/gait_speed.hpp"
#include "gait_speed_ros2/orchestrators/gait_speed_states.hpp"

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
  blackboard->clear();
  blackboard->set("node", node);

  // orchestrator
  auto gait_speed_node = std::make_shared<gait_speed::GaitSpeed>(blackboard);
  gait_speed_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  gait_speed_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // behaviors
  std::vector<std::string> plugins;
  
  plugins = {
    "is_pointing_bt_node",
    "spin_bt_node",
    "identify_bt_node",
    "navigate_to_bt_node",
    "save_detection_in_bb_bt_node"
  };

  auto find_person_node = std::make_shared<gait_speed::BehaviorRunner>(
    blackboard,
    "find_patient",
    "/bt_xml/find_patient.xml",
    plugins
  );

  plugins = {
    "speak_bt_node",
    "listen_bt_node",
    "dialog_confirmation_bt_node",
    "identify_bt_node",
    "is_in_front_bt_node",
    "activate_attention_bt_node",
    "deactivate_attention_bt_node",
    "get_detection_from_bb_bt_node",
    "save_detection_in_bb_bt_node"
  };

  auto explain_gait_speed_node = std::make_shared<gait_speed::BehaviorRunner>(
    blackboard,
    "explain_gait_speed",
    "/bt_xml/explain.xml",
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
    "is_detected_bt_node",
    "activate_attention_bt_node",
    "deactivate_attention_bt_node",
    "get_detection_from_bb_bt_node",
    "speak_bt_node",
  };

  auto measure_node = std::make_shared<gait_speed::BehaviorRunner>(
    blackboard,
    "measure_gait_speed",
    "/bt_xml/measure_gait_speed.xml",
    plugins
  );

  plugins = {
    "speak_bt_node",
  };

  auto error_node = std::make_shared<gait_speed::BehaviorRunner>(
    blackboard,
    "error",
    "/bt_xml/error.xml",
    plugins
  );

  exec.add_node(gait_speed_node->get_node_base_interface());
  
  // exec.add_node(node->get_node_base_interface());

  exec.add_node(find_person_node->get_node_base_interface());
  exec.add_node(explain_gait_speed_node->get_node_base_interface());
  exec.add_node(measure_node->get_node_base_interface());
  exec.add_node(error_node->get_node_base_interface());

  gait_speed_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  find_person_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  explain_gait_speed_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  measure_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  error_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  RCLCPP_INFO(node->get_logger(), "gait_speed_node state: %s", gait_speed_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "find_person_node state: %s", find_person_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "explain_gait_speed_node state: %s", explain_gait_speed_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "measure_node state: %s", measure_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "error_node state: %s", error_node->get_current_state().label().c_str());
  
  while (rclcpp::ok()) {
    exec.spin_some();
    
    if (gait_speed_node->get_state() == gait_speed::State::STOP) {
      RCLCPP_INFO(node->get_logger(), "Orchestrator stopped. Exiting...");
      
      gait_speed_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

      break;
    }
  }
  RCLCPP_INFO(node->get_logger(), "gait_speed_node state: %s", gait_speed_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "find_person_node state: %s", find_person_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "explain_gait_speed_node state: %s", explain_gait_speed_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "measure_node state: %s", measure_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "error_node state: %s", error_node->get_current_state().label().c_str());

  try {
    float result = blackboard->get<float>("gait_speed_result");
    RCLCPP_INFO(node->get_logger(), "\n*************************************************************\nGait speed result: %.2f seconds", result);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Gait speed result not calculated");
  }
  
  rclcpp::shutdown();

  return 0;
}
