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


#ifndef EXAMPLE_REPEAT_HPP_
#define EXAMPLE_REPEAT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace example
{

using namespace std::chrono_literals;
using std::placeholders::_1;

class ExampleRepeat : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  ExampleRepeat(BT::Blackboard::Ptr blackboard);
  int get_state() {return state_;}

  static const int LISTEN = 0;
  static const int REPEAT = 1;
  static const int STOP = 2;

private:
  void control_cycle();
  void status_callback(std_msgs::msg::String::UniquePtr msg);
  void go_to_state(int state);
  bool check_behavior_finished();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  int state_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  std::string status_received_, last_status_;
  BT::Blackboard::Ptr blackboard_;

  bool started_ = false;
  
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr result_pub_;

};

} // namespace example

#endif  // EXAMPLE_REPEAT_HPP_
