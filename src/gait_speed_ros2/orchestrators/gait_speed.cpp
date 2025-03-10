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

namespace gait_speed
{

GaitSpeed::GaitSpeed(BT::Blackboard::Ptr blackboard)
: CascadeLifecycleNode("gait_speed_node"),
  state_(State::INIT),
  n_tries_(0),
  last_status_(""),
  blackboard_(blackboard)
{
  this->declare_parameter<float>("value", 4.0);
  this->declare_parameter<std::string>("mode", "distance");

  double value = this->get_parameter("value").as_double();
  std::string mode = this->get_parameter("mode").as_string();

  if (mode == "distance") {
    RCLCPP_INFO(get_logger(), "GaitSpeed constructor: %f meters", value);
  } else if (mode == "time") {
    RCLCPP_INFO(get_logger(), "GaitSpeed constructor: %f seconds", value);
  }
  blackboard_->set("target", value);
  blackboard_->set("mode", mode); // to create a generic TargetReached BT node

  status_sub_ = create_subscription<std_msgs::msg::String>(
    "behavior_status", 10, std::bind(&GaitSpeed::status_callback, this, _1));

  result_pub_ = create_publisher<std_msgs::msg::Float64>("gait_speed_result", 10);

  go_to_state(state_);
}

void
GaitSpeed::status_callback(std_msgs::msg::String::UniquePtr msg)
{
  last_status_ = msg.get()->data;
  RCLCPP_DEBUG(get_logger(), "Status received: %s", last_status_.c_str());
  if (last_status_ == "DEACTIVATED") {
    last_status_ = "";
  }
}

void
GaitSpeed::control_cycle()
{
  std_msgs::msg::Float64 result_msg;

  switch (state_) {
    case State::INIT:
      go_to_state(State::FIND);
      break;
    case State::FIND:
      if(last_status_ == "") {
        RCLCPP_INFO_ONCE(get_logger(), "BT not started yet");
        break;
      }
      if (check_behavior_finished()) {
        if (last_status_ == "SUCCESS") {
          RCLCPP_INFO(get_logger(), "[State - FIND]: Patient found");
          go_to_state(State::EXPLAIN);
        } else {
          RCLCPP_INFO(get_logger(), "[State - FIND]: Stopping FSM");
          go_to_state(State::CLEAN);
        }
      }
      break;
    case State::EXPLAIN:
      if (check_behavior_finished()) {
        if (last_status_ == "FAILURE") {
          RCLCPP_INFO(get_logger(), "[State - EXPLAIN]: Error providing  instructions");
          go_to_state(State::ERROR);
        } else {
          RCLCPP_INFO(get_logger(), "[State - EXPLAIN]: Instructions given");
          go_to_state(State::MEASURE);
        }
      }
      break;
    case State::MEASURE:
      if (check_behavior_finished()) {
        if (last_status_ == "FAILURE") {
          RCLCPP_INFO(get_logger(), "[State - MEASURE]: Error measuring gait speed");
          go_to_state(State::ERROR);
        } else {
          blackboard_->get("gait_speed_result", result_msg.data);
          RCLCPP_INFO(get_logger(), "[State - MEASURE]: Gait speed test finished (%.2f)", result_msg.data);
          result_pub_->publish(result_msg);
          go_to_state(State::CLEAN);
        }
      }
      break;
    case State::ERROR:
      if (check_behavior_finished()) {
        RCLCPP_INFO(get_logger(), "[State - ERROR]: Error detected");
        go_to_state(State::CLEAN);
      }
      break;
    case State::CLEAN:
      RCLCPP_INFO(get_logger(), "[State - CLEAN]: Cleaning up");
      go_to_state(State::STOP);
      break;
    case State::STOP:
      break;
    default:
      RCLCPP_INFO(get_logger(), "[State - ERROR]: Cleaning up");
      go_to_state(State::CLEAN);
      break;
  }
}

void
GaitSpeed::go_to_state(State state)
{
  state_ = state;

  switch (state_) {
    case State::INIT:
      RCLCPP_INFO(get_logger(), "State: INIT");
      break;
    case State::FIND:
      RCLCPP_INFO(get_logger(), "State: FIND");
      add_activation("find_patient");
      break;
    case State::EXPLAIN:
      RCLCPP_INFO(get_logger(), "State: EXPLAIN");
      remove_activation("find_patient");
      add_activation("explain_gait_speed");
      break;
    case State::MEASURE: // TODO: add a check to see if the robot is moving to activate a different behavior
      RCLCPP_INFO(get_logger(), "State: MEASURE");
      remove_activation("explain_gait_speed");
      add_activation("measure_gait_speed");
      break;
    case State::CLEAN:
      RCLCPP_INFO(get_logger(), "State: CLEAN");
      remove_activation("measure_gait_speed");
      remove_activation("error");
      break;
    case State::ERROR:
      clear_activation();
      RCLCPP_INFO(get_logger(), "State: ERROR");
      add_activation("error");
      break;
    case State::STOP:
      RCLCPP_INFO(get_logger(), "State: STOP");
      clear_activation();
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown state in gait speed test");
      break;
  }
}

bool
GaitSpeed::check_behavior_finished()
{
  RCLCPP_DEBUG(get_logger(), "State %d. Checking behavior finished: %s",  static_cast<int>(state_), last_status_.c_str());
  return last_status_ == "FAILURE" || last_status_ == "SUCCESS";
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GaitSpeed::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "GaitSpeed on_activate");

  timer_ =
    create_wall_timer(50ms, std::bind(&GaitSpeed::control_cycle, this));

  result_pub_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GaitSpeed::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "GaitSpeed on_deactivate");

  timer_ = nullptr;

  result_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace gait_speed
