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

namespace gait_speed
{

GaitSpeedSimple::GaitSpeedSimple(BT::Blackboard::Ptr blackboard)
: CascadeLifecycleNode("gait_speed_node"),
  state_(State::INIT),
  last_status_(""),
  blackboard_(blackboard)
{
  this->declare_parameter<float>("value", 4.0);
  double value = this->get_parameter("value").as_double();

  RCLCPP_INFO(get_logger(), "GaitSpeedSimple constructor: %f meters", value);
  
  blackboard_->set("target", value);

  status_sub_ = create_subscription<std_msgs::msg::String>(
    "behavior_status", 10, std::bind(&GaitSpeedSimple::status_callback, this, _1));

  result_pub_ = create_publisher<std_msgs::msg::Float64>("gait_speed_result", 10);

  go_to_state(state_);
}

void
GaitSpeedSimple::status_callback(std_msgs::msg::String::UniquePtr msg)
{
  last_status_ = msg.get()->data;
  
  if (!started_) {
    started_ = true;
  }
}

void
GaitSpeedSimple::control_cycle()
{
  std_msgs::msg::Float64 result_msg;

  if (!started_) {
    RCLCPP_INFO(get_logger(), "Not started yet");
    return;
  }

  switch (state_) {
    case State::INIT:
      if (check_behavior_finished()) {
          go_to_state(State::FIND);
      } else {
          go_to_state(State::STOP);
      }
      break;
    case State::FIND:
      if (last_status_ == "SUCCESS") {
        go_to_state(State::MEASURE);
      } else {
        go_to_state(State::STOP);
      }
      break;
    case State::MEASURE:
      if (check_behavior_finished()) {
        go_to_state(State::STOP);
      }
      break;
    case State::STOP:
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
GaitSpeedSimple::go_to_state(State state)
{
  clear_activation();
  state_ = state;

  switch (state_) {
    case State::FIND:
      RCLCPP_INFO(get_logger(), "State: State::FIND");
      add_activation("State::FIND_person");
      // on_activate(get_current_state());
      break;
    case State::MEASURE:
      RCLCPP_INFO(get_logger(), "State: State::MEASURE");
      add_activation("State::MEASURE_gait_speed_dist");
      // on_activate(get_current_state());
      break;
    default:
      state_ = State::STOP;
      RCLCPP_INFO(get_logger(), "Deactivating");
      cleanup();
      deactivate();
      break;
  }
}

bool
GaitSpeedSimple::check_behavior_finished()
{
  return last_status_ != "RUNNING";
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GaitSpeedSimple::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "GaitSpeedSimple on_activate");

  timer_ =
    create_wall_timer(50ms, std::bind(&GaitSpeedSimple::control_cycle, this));

  result_pub_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GaitSpeedSimple::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "GaitSpeedSimple on_deactivate");

  timer_ = nullptr;

  result_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace gait_speed
