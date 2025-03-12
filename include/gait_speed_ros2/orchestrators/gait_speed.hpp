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


#ifndef GAITSPEED_HPP_
#define GAITSPEED_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "gait_speed_ros2/orchestrators/gait_speed_states.hpp"

namespace gait_speed
{

using namespace std::chrono_literals;
using std::placeholders::_1;

class GaitSpeed : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  GaitSpeed(BT::Blackboard::Ptr blackboard);
  State get_state() { return state_; }

private:
  void control_cycle();
  void status_callback(std_msgs::msg::String::UniquePtr msg);
  void go_to_state(State state);
  bool check_behavior_finished();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  State state_;
  int n_tries_, n_tries_counter_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  std::string status_received_, last_status_;
  BT::Blackboard::Ptr blackboard_;
  
  bool robot_moves_;

  bool started_ = false;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr result_pub_;

};

} // namespace gait_speed

#endif  // GAITSPEED_HPP_
