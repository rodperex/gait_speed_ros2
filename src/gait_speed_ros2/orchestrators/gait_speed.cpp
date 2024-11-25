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

namespace gait_speed
{

GaitSpeed::GaitSpeed(BT::Blackboard::Ptr blackboard)
: CascadeLifecycleNode("gait_speed_node"),
  state_(INIT),
  last_status_(""),
  blackboard_(blackboard)
{
  this->declare_parameter<float>("value", 4.0);
  this->declare_parameter<std::string>("mode", "distance");
  this->declare_parameter<bool>("robot_moves", false);

  mode_ = this->get_parameter("mode").as_string();
  double value = this->get_parameter("value").as_double();
  robot_moves_ = this->get_parameter("robot_moves").as_bool();

  if (mode_ == "distance") {
    RCLCPP_INFO(get_logger(), "GaitSpeed constructor: %f meters", value);
  } else if (mode_ == "time") {
    RCLCPP_INFO(get_logger(), "GaitSpeed constructor: %f seconds", value);
  }
  blackboard_->set("target", value);
  blackboard_->set("mode", mode_); // to create a generic TargetReached BT node

  status_sub_ = create_subscription<std_msgs::msg::String>(
    "behavior_status", 10, std::bind(&GaitSpeed::status_callback, this, _1));

  result_pub_ = create_publisher<std_msgs::msg::Float64>("gait_speed_result", 10);

  go_to_state(state_);
}

void
GaitSpeed::status_callback(std_msgs::msg::String::UniquePtr msg)
{
  last_status_ = msg.get()->data;
  
  if (!started_) {
    started_ = true;
  }
}

void
GaitSpeed::control_cycle()
{
  std_msgs::msg::Float64 result_msg;

  if (!started_) {
    RCLCPP_INFO(get_logger(), "Not started yet");
    return;
  }

  switch (state_) {
    case INIT:
      if (check_behavior_finished()) {
        if (last_status_ == "SUCCESS") {
          started_ = false;
          go_to_state(FIND);
        } else {
          go_to_state(STOP);
        }
      }
      break;
    case FIND:
      if (last_status_ == "SUCCESS") {
        started_ = false;
        go_to_state(EXPLAIN);
      } else {
        go_to_state(STOP);
      }
      break;
    case EXPLAIN:
      if (last_status_ == "SUCCESS") {
        started_ = false;
        // go_to_state(PREPARE);
        go_to_state(MEASURE);
      } else {
        go_to_state(STOP);
      }
      break;
    // case PREPARE:
    //   if (last_status_ == "SUCCESS") {
    //     started_ = false;
    //     go_to_state(MEASURE);
    //   } else {
    //     go_to_state(STOP);
    //   }
    //   break;
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
GaitSpeed::go_to_state(int state)
{
  clear_activation();
  state_ = state;

  switch (state_) {
    case FIND:
      RCLCPP_INFO(get_logger(), "State: FIND");
      add_activation("find_person");
      on_activate(get_current_state());
      break;
    case EXPLAIN:
      RCLCPP_INFO(get_logger(), "State: EXPLAIN");
      add_activation("explain_gait_speed");
      on_activate(get_current_state());
      break;
    // case PREPARE:
    //   RCLCPP_INFO(get_logger(), "State: PREPARE");
    //   add_activation("prepare_gait_speed");
    //   on_activate(get_current_state());
    //   break;
    case MEASURE:
      RCLCPP_INFO(get_logger(), "State: MEASURE");
      if (robot_moves_) {
        add_activation("measure_gait_speed_dist");
        on_activate(get_current_state());
      } else {
        add_activation("measure_gait_speed_dist_static");
        on_activate(get_current_state());
      }
      break;
    default:
      state_ = STOP;
      RCLCPP_INFO(get_logger(), "Deactivating");
      cleanup();
      deactivate();
      // on_deactivate(get_current_state());
      // rclcpp::shutdown();
      break;
  }
}

bool
GaitSpeed::check_behavior_finished()
{
  return last_status_ != "RUNNING";
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
