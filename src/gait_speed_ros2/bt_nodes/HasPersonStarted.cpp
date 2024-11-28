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


#include "gait_speed_ros2/bt_nodes/HasPersonStarted.hpp"

namespace gait_speed
{
HasPersonStarted::HasPersonStarted(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  min_distance_(0.2),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  getInput("min_distance", min_distance_);
}

BT::NodeStatus
HasPersonStarted::tick()
{
  if (get_distance_travelled() < min_distance_) {
    config().blackboard->set("start_time", node_->now());
    return BT::NodeStatus::FAILURE;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

float
HasPersonStarted::get_distance_travelled()
{
  try
    {
      tf2::Stamped<tf2::Transform> odom2patient_at_start;
      config().blackboard->get("map2start", odom2patient_at_start);

      auto odom2patient_msg = tf_buffer_.lookupTransform("odom", "patient", tf2::TimePointZero);
      tf2::Stamped<tf2::Transform> odom2patient;
      tf2::fromMsg(odom2patient_msg, odom2patient);

      auto person_at_start2person = odom2patient_at_start.inverse() * odom2patient;
      return person_at_start2person.getOrigin().length();
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
      return -1;
    }
  return 0.0;
}

}  // namespace gait_speed

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<gait_speed::HasPersonStarted>("HasPersonStarted");
}
