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


#ifndef IS_MY_PERSON_HPP_
#define IS_MY_PERSON_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "perception_system/PerceptionListener.hpp"
#include "perception_system_interfaces/msg/detection.hpp"
#include "perception_system/PerceptionUtils.hpp"

namespace gait_speed
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using pl = perception_system::PerceptionListener;

class IsMyPerson : public BT::ConditionNode
{
public:
  IsMyPerson(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::int64_t>("person_id"),
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;

  std::int64_t person_id_;
  float threshold_;

};

} // namespace gait_speed

#endif  // IS_MY_PERSON_HPP_
