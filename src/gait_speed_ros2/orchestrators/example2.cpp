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


#include "gait_speed_ros2/orchestrators/example2.hpp"

namespace example
{

Example2::Example2(BT::Blackboard::Ptr blackboard)
: CascadeLifecycleNode("example2_node"),
  state_(LISTEN),
  last_status_(""),
  blackboard_(blackboard)
{

  status_sub_ = create_subscription<std_msgs::msg::String>(
    "behavior_status", 10, std::bind(&Example2::status_callback, this, _1));

  result_pub_ = create_publisher<std_msgs::msg::Float64>("result", 10);
  
  go_to_state(state_);

  RCLCPP_INFO(get_logger(), "Example2 created. State: %d", state_);

}

void
Example2::status_callback(std_msgs::msg::String::UniquePtr msg)
{
  RCLCPP_DEBUG(get_logger(), "Status received: %s", msg.get()->data.c_str());

  last_status_ = msg.get()->data;
  
  if (!started_) {
    RCLCPP_INFO(get_logger(), "Started");
    started_ = true;
  }
  
}

void
Example2::control_cycle()
{
  std_msgs::msg::Float64 result_msg;


  RCLCPP_DEBUG(get_logger(), "Example2 control cycle. State: %d. Last status: %s", state_, last_status_.c_str());

  if (!started_) {
    RCLCPP_INFO(get_logger(), "Not started yet");
    return;
  }

  switch (state_) {
    case LISTEN:
    RCLCPP_DEBUG(get_logger(), "LISTEN");
      if (check_behavior_finished()) {
        if (last_status_ == "SUCCESS") {
          started_ = false;
          RCLCPP_INFO(get_logger(), "LISTEN finished successfully");
          go_to_state(REPEAT);
        } else {
          RCLCPP_ERROR(get_logger(), "LISTEN failed");
          go_to_state(STOP);
        }
      }
      break;
    case REPEAT:
    RCLCPP_DEBUG(get_logger(), "REPEAT");
      if (check_behavior_finished()) {
        if (last_status_ == "SUCCESS") {
          started_ = false;
          RCLCPP_INFO(get_logger(), "REPEAT finished successfully");
          go_to_state(STOP);
        } else {
          RCLCPP_ERROR(get_logger(), "REPEAT failed");
          go_to_state(STOP);
        }
      }
      break;
    case STOP:
      if (last_status_ == "SUCCESS") {
        RCLCPP_INFO(get_logger(), "Example2 finished successfully.");
        result_msg.data = 0;
      } else {
        result_msg.data = -1;
        RCLCPP_ERROR(get_logger(), "Example2 test failed");
      }
      result_pub_->publish(result_msg);
      break;
    default:
      break;
  }
}

void
Example2::go_to_state(int state)
{
  clear_activation();

  RCLCPP_INFO(get_logger(), "Activations cleared");

  state_ = state;

  RCLCPP_INFO(get_logger(), "Going to state %d", state_);

  switch (state_) {
    case REPEAT:
      add_activation("repeat");
      on_activate(get_current_state());
      RCLCPP_INFO(get_logger(), "REPEAT activated");
      state_ = REPEAT;
      break;
    case LISTEN:
      add_activation("listen");
      on_activate(get_current_state());
      RCLCPP_INFO(get_logger(), "LISTEN activated");
      state_ = LISTEN;
      break;
    default:
      state_ = STOP;
      RCLCPP_INFO(get_logger(), "Deactivating");
      // deactivate();
      // trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      on_deactivate(get_current_state());
      break;
  }
}

bool
Example2::check_behavior_finished()
{
  return last_status_ != "RUNNING";
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Example2::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Example2 on_activate");

  RCLCPP_INFO(get_logger(), "Initial state: %d", state_);

  timer_ =
    create_wall_timer(50ms, std::bind(&Example2::control_cycle, this));
  
  RCLCPP_INFO(get_logger(), "Timer created");

  result_pub_->on_activate();

  // Subscribers cannot be activated or deactivated

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Example2::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Example2 on_deactivate");

  timer_ = nullptr;

  result_pub_->on_deactivate();

  // trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);


  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace example
