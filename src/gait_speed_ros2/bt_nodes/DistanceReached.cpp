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


#include "gait_speed_ros2/bt_nodes/DistanceReached.hpp"

namespace gait_speed
{
DistanceReached::DistanceReached(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  distance_(0.0),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
  config().blackboard->get("distance", distance_);
}

BT::NodeStatus
DistanceReached::tick()
{
  try
  {
    tf2::Stamped<tf2::Transform> odom2bf0;
    config().blackboard->get("odom2bf0", odom2bf0);

    auto odom2bf_msg = tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
    tf2::Stamped<tf2::Transform> odom2bf;
    tf2::fromMsg(odom2bf_msg, odom2bf);

    auto bf02bf = odom2bf0.inverse() * odom2bf;
    float distance_traveled = bf02bf.getOrigin().length();

    if (distance_traveled >= distance_) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
  catch (tf2::TransformException &ex)
  {
      RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
      return BT::NodeStatus::FAILURE;
  }
}
}  // namespace gait_speed

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<gait_speed::DistanceReached>("DistanceReached");
}
