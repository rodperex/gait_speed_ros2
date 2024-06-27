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


#ifndef BEHAVIOR_RUNNER__HPP_
#define BEHAVIOR_RUNNER__HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/string.hpp"

namespace gait_speed
{

using namespace std::chrono_literals;

class BehaviorRunner : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  BehaviorRunner(
    BT::Blackboard::Ptr blackboard,
    const std::string &name,
    const std::string &xml_path,
    const std::vector<std::string> &plugins);

private:
  void control_cycle();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::SharedPtr node_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;

};

} // namespace gait_speed

#endif  // BEHAVIOR_RUNNER__HPP_
