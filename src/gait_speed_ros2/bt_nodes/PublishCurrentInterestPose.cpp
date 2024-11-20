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


#include "gait_speed_ros2/bt_nodes/PublishCurrentInterestPose.hpp"

namespace gait_speed
{
PublishCurrentInterestPose::PublishCurrentInterestPose(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  interest_(""),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
  getInput("interest", interest_);
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

BT::NodeStatus
PublishCurrentInterestPose::tick()
{
  
  try
  {
    // Get the transform from map to the interest
    auto map2interest_msg = tf_buffer_.lookupTransform("map", interest_, tf2::TimePointZero);
    tf2::Stamped<tf2::Transform> map2interest;
    tf2::fromMsg(map2interest_msg, map2interest);

    // Publish the pose
    auto pose = tf2::toMsg(map2interest);
    static_broadcaster_->sendTransform(pose);
    
    return BT::NodeStatus::SUCCESS;
  }
  catch(const std::exception& ex)
  {
    RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
  
  return BT::NodeStatus::RUNNING;
}


}  // namespace gait_speed

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<gait_speed::PublishCurrentInterestPose>("PublishCurrentInterestPose");
}

