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


#ifndef GAITSPEED__DIST_HPP_
#define GAITSPEED__DIST_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "std_msgs/msg/bool.hpp"

namespace gait_speed
{

using namespace std::chrono_literals;
using std::placeholders::_1;

class GaitSpeedDist : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  GaitSpeedDist();

private:
  void control_cycle();
  void status_callback(std_msgs::msg::Bool::UniquePtr msg);
  void go_to_state(int state);
  bool check_init_2_explain();
  bool check_explain_2_prepare();
  bool check_prepare_2_verify_prepare();
  bool check_verify_prepare_2_start();
  bool check_start_2_walk();
  bool check_walk_2_stop();
  

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  int state_;
  static const int INIT = 0;
  static const int EXPLAIN = 1;
  static const int PREPARE = 2;
  static const int VERIFY_PREPARE = 3;
  static const int START = 4;
  static const int WALK = 5;
  static const int STOP = 6;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  bool status_received_, last_status_;

};

} // namespace gait_speed

#endif  // GAITSPEED__DIST_HPP_
