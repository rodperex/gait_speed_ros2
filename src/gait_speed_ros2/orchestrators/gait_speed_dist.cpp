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

namespace gait_speed
{

GaitSpeedDist::GaitSpeedDist(BT::Blackboard::Ptr blackboard)
: CascadeLifecycleNode("gait_speed_test"),
  state_(INIT),
  status_received_("FAILURE")
{
  RCLCPP_INFO(get_logger(), "GaitSpeedDist constructor");

  status_sub_ = create_subscription<std_msgs::msg::String>(
    "behavior_status", 10, std::bind(&GaitSpeedDist::status_callback, this, _1));

  result_pub_ = create_publisher<std_msgs::msg::Float64>("gait_speed_result", 10);

  blackboard_ = blackboard;
}

void 
GaitSpeedDist::status_callback(std_msgs::msg::String::UniquePtr msg)
{
  last_status_ = msg.get()->data;
}

void
GaitSpeedDist::control_cycle()
{
  std_msgs::msg::Float64 result_msg;

  switch (state_) {
    case INIT:
      if (check_behavior_finished()) {
        if (last_status_ == "SUCCESS") {
          go_to_state(FIND);
        } else {
          go_to_state(STOP);
        }
      } 
      break;
    case FIND:
      if (last_status_ == "SUCCESS") {
          go_to_state(EXPLAIN);
      } else {
          go_to_state(STOP);
      }
      break;
    case EXPLAIN:
      if (last_status_ == "SUCCESS") {
          go_to_state(PREPARE);
      } else {
          go_to_state(STOP);
      }
      break;
    case PREPARE:
      if (last_status_ == "SUCCESS") {
          go_to_state(MEASURE);
      } else {
          go_to_state(STOP);
      }
      break;
    case MEASURE:
      if (check_behavior_finished()) {
        go_to_state(STOP);
      } 
      break;
    case STOP:
      clear_activation();
      if (last_status_ == "SUCCESS") {
        blackboard_->get("gait_speed_time", result_msg.data);
        RCLCPP_INFO(get_logger(), "Gait speed test finished successfully");
      } else {
        result_msg.data = -1;
        RCLCPP_ERROR(get_logger(), "Gait speed test failed");
      }
      result_pub_->publish(result_msg);
      break;
    default:
      break;
  }
}

void
GaitSpeedDist::go_to_state(int state)
{
  clear_activation();
  state_ = state;

  switch (state_) {
    case FIND:
      add_activation("find_person");
      break;
    case EXPLAIN:
      add_activation("explain_gait_speed");
      break;
    case PREPARE:
      add_activation("prepare_gait_speed");
      break;
    case MEASURE:
      add_activation("measure_gait_speed_dist");
      break;
    default:
      break;
  }
}

bool
GaitSpeedDist::check_behavior_finished()
{
  return last_status_ != "RUNNING";
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GaitSpeedDist::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "GaitSpeedDist on_activate");

  timer_ =
    create_wall_timer(50ms, std::bind(&GaitSpeedDist::control_cycle, this));

  result_pub_->on_activate();
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GaitSpeedDist::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "GaitSpeedDist on_deactivate");

  timer_ = nullptr;

  result_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace gait_speed
