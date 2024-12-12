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
  this->declare_parameter<std::string>("mode", "distance");
  
  double value = this->get_parameter("value").as_double();
  std::string mode = this->get_parameter("mode").as_string();

  RCLCPP_INFO(get_logger(), "GaitSpeedSimple constructor: %.2f meters", value);
  
  blackboard_->set("target", value);
  blackboard_->set("mode", mode);

  status_sub_ = create_subscription<std_msgs::msg::String>(
    "behavior_status", 10, std::bind(&GaitSpeedSimple::status_callback, this, _1));

  result_pub_ = create_publisher<std_msgs::msg::Float64>("gait_speed_result", 10);

  go_to_state(state_);
}

void
GaitSpeedSimple::status_callback(std_msgs::msg::String::UniquePtr msg)
{
  last_status_ = msg.get()->data;
  RCLCPP_DEBUG(get_logger(), "Status received: %s", last_status_.c_str());
  if (last_status_ == "DEACTIVATED") {
    last_status_ = "";
  }
}

void
GaitSpeedSimple::control_cycle()
{
  std_msgs::msg::Float64 result_msg;

  switch (state_) {
    case State::INIT:
      go_to_state(State::FIND);
      break;
    case State::FIND:
      if(last_status_ == "") {
        RCLCPP_INFO(get_logger(), "BT not started yet");
        break;
      }
      if (last_status_ == "SUCCESS") {
        RCLCPP_INFO(get_logger(), "Patient found");
        go_to_state(State::MEASURE);
      } else if (last_status_ == "FAILURE") {
        RCLCPP_INFO(get_logger(), "Stopping FSM");
        go_to_state(State::STOP);
      }
      break;
    case State::MEASURE:
      if (check_behavior_finished()) {
        RCLCPP_INFO(get_logger(), "Gait speed test finished");
        go_to_state(State::STOP);
      }
      break;
    case State::STOP:
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
  // clear_activation();
  state_ = state;
  last_status_ = "";

  RCLCPP_INFO(get_logger(), "Going to state %d.", static_cast<int>(state_));

  switch (state_) {
    case State::INIT:
      RCLCPP_INFO(get_logger(), "State: INIT");
      break;
    case State::FIND:
      RCLCPP_INFO(get_logger(), "State: FIND");
      add_activation("find_patient");
      // on_activate(get_current_state());
      break;
    case State::MEASURE:
      RCLCPP_INFO(get_logger(), "State: MEASURE");
      remove_activation("find_patient");
      add_activation("measure_gait_speed");
      // on_activate(get_current_state());
      break;
    case State::STOP:
      RCLCPP_INFO(get_logger(), "State: STOP");
      remove_activation("measure_gait_speed");
      // cleanup();
      // deactivate();
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown state in gait speed test");
      break;
  }
}

bool
GaitSpeedSimple::check_behavior_finished()
{
  RCLCPP_DEBUG(get_logger(), "State %d. Checking behavior finished: %s",  static_cast<int>(state_), last_status_.c_str());
  return last_status_ == "FAILURE" || last_status_ == "SUCCESS";
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
