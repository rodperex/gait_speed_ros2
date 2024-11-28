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


#ifndef HAS_PERSON_STARTED_HPP_
#define HAS_PERSON_STARTED_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace gait_speed
{

using namespace std::chrono_literals;
using std::placeholders::_1;

class HasPersonStarted : public BT::ConditionNode
{
public:
  HasPersonStarted(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);
  
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<float>("min_distance", "Minimum distance to consider the person has started the test"),
      });
  }

private:
  float get_distance_travelled();

  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  
  float min_distance_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;  
};

} // namespace gait_speed

#endif  // HAS_PERSON_STARTED_HPP_
