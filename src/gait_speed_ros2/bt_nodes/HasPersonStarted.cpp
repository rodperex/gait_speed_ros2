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
  // config().blackboard->get("person_frame", frame_);

  getInput("min_distance", min_distance_);
  getInput("person_frame", frame_);
}

BT::NodeStatus
HasPersonStarted::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "HAS_PERSON_STARTED");
  RCLCPP_INFO(node_->get_logger(), "Distance travelled (min = %.2f): %.2f", min_distance_, get_distance_travelled());

  if (get_distance_travelled() < min_distance_) {
    RCLCPP_DEBUG(node_->get_logger(), "Person has not started yet. Distance travelled: %.2f", get_distance_travelled());
    config().blackboard->set("start_time", node_->now());
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Person has started the test. Moved %.2f meters.", get_distance_travelled());
    return BT::NodeStatus::SUCCESS;
  }
}

float
HasPersonStarted::get_distance_travelled()
{
  try
    {
      tf2::Stamped<tf2::Transform> map2start;
      config().blackboard->get("map2start", map2start);

      RCLCPP_DEBUG(node_->get_logger(), "Getting distance travelled for frame %s", frame_.c_str());

      auto map2person_msg = tf_buffer_.lookupTransform("map", frame_, tf2::TimePointZero);
      tf2::Stamped<tf2::Transform> map2person; // Current position of the person
      tf2::fromMsg(map2person_msg, map2person);

      auto start2person = map2start.inverse() * map2person;
      return start2person.getOrigin().length();
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
