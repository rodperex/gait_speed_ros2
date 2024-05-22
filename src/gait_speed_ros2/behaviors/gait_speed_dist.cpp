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


#include "gait_speed_ros2/behaviors/gait_speed_dist.hpp"

namespace gait_speed
{

GaitSpeedDist::GaitSpeedDist()
: CascadeLifecycleNode("gait_speed_test"),
  state_(INIT),
  status_received_(false)
{
  RCLCPP_INFO(get_logger(), "GaitSpeedDist constructor");

  status_sub_ = create_subscription<std_msgs::msg::Bool>(
    "gait_speed_status", 10, std::bind(&GaitSpeedDist::status_callback, this, _1));
}

void 
GaitSpeedDist::status_callback(std_msgs::msg::Bool::UniquePtr msg)
{
  last_status_ = msg.get()->data;
}

void
GaitSpeedDist::control_cycle()
{
  switch (state_) {
    case INIT:
      if (check_init_2_explain()) {
        go_to_state(EXPLAIN);
      }
      break;
    case EXPLAIN:
      if (check_explain_2_prepare()) {
        go_to_state(PREPARE);
      }
      break;
    case PREPARE:
      if (check_prepare_2_verify_prepare()) {
        go_to_state(VERIFY_PREPARE);
      }
      break;
    case VERIFY_PREPARE:
      if (check_verify_prepare_2_start()) {
        go_to_state(START);
      }
      break;
    case START:
      add_activation("measure_gait_speed_dist");
      if (check_start_2_walk()) {
        go_to_state(WALK);
      }
      break;
    case WALK:
      if (check_walk_2_stop()) {
        go_to_state(STOP);
      }
      break;
    case STOP:
      if (last_status_) {
        RCLCPP_INFO(get_logger(), "Gait speed test finished successfully");
      } else {
        RCLCPP_ERROR(get_logger(), "Gait speed test failed");
      }
      break;
    default:
      break;
  }
}

void
GaitSpeedDist::go_to_state(int state)
{
  state_ = state;
}

bool
GaitSpeedDist::check_init_2_explain()
{
  return true;
}

bool
GaitSpeedDist::check_explain_2_prepare()
{
  return true;
}

bool
GaitSpeedDist::check_prepare_2_verify_prepare()
{
  return true;
}

bool
GaitSpeedDist::check_verify_prepare_2_start()
{
  return true;
}

bool
GaitSpeedDist::check_start_2_walk()
{
  return true;
}

bool
GaitSpeedDist::check_walk_2_stop()
{
  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GaitSpeedDist::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "GaitSpeedDist on_activate");

  timer_ =
    create_wall_timer(50ms, std::bind(&GaitSpeedDist::control_cycle, this));
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GaitSpeedDist::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "GaitSpeedDist on_deactivate");

  timer_ = nullptr;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace gait_speed
