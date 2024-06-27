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


#ifndef DISTANCE_REACHED_HPP_
#define DISTANCE_REACHED_HPP_

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace gait_speed
{

using namespace std::chrono_literals;
using std::placeholders::_1;

class DistanceReached : public BT::ConditionNode
{
public:
  DistanceReached(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

private:

};

} // namespace gait_speed

#endif  // DISTANCE_REACHED_HPP_
