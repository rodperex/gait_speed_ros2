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


#include "gait_speed_ros2/bt_nodes/CheckEndTest.hpp"

namespace gait_speed
{
CheckEndTest::CheckEndTest(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  target_(0.0),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
  config().blackboard->get("target", target_); // aquí está el error

  getInput("frame_name", frame_name_);
  getInput("mode", mode_);
}

BT::NodeStatus
CheckEndTest::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "CHECK_END_TEST");
  RCLCPP_DEBUG(node_->get_logger(), "Target: %.2f, Mode: %s.", target_, mode_.c_str());
  rclcpp::spin_some(node_->get_node_base_interface());

  float current;
  if (mode_ == "distance") {
    current = get_distance_travelled();
    RCLCPP_INFO(node_->get_logger(), "[CHECK_END_TEST]: Distance travelled: %.2f m. Target: %.2f m", get_distance_travelled(), target_);
    if (current >= target_) {
      RCLCPP_INFO(node_->get_logger(), "[CHECK_END_TEST]: Target reached. Finishing test.");
      config().blackboard->set("time_elapsed", get_time_elapsed());
      return BT::NodeStatus::SUCCESS;
    }
  } else if (mode_ == "time") {
    current = get_time_elapsed();
    if (current >= target_) {
      config().blackboard->set("distance_travelled", get_distance_travelled());
      return BT::NodeStatus::SUCCESS;
    }
  }
  RCLCPP_DEBUG(node_->get_logger(), "Target (%.2f/%.2f) not reached. Continuing test.", current, target_);
  return BT::NodeStatus::RUNNING;
}

float
CheckEndTest::get_distance_travelled()
{
  try
    {
      tf2::Stamped<tf2::Transform> map2person_at_start;
      config().blackboard->get("map2start", map2person_at_start);

      auto map2person_msg = tf_buffer_.lookupTransform("map", frame_name_, tf2::TimePointZero);
      tf2::Stamped<tf2::Transform> map2person;
      tf2::fromMsg(map2person_msg, map2person);

      auto person_at_start2person = map2person_at_start.inverse() * map2person;

      tf_time_ = rclcpp::Time(map2person_msg.header.stamp, RCL_STEADY_TIME);

      return person_at_start2person.getOrigin().length();
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(node_->get_logger(), "[CHECK_END_TEST]: %s", ex.what());
      return -1.0;
    }
  return 0.0;
}

float
CheckEndTest::get_time_elapsed()
{
  rclcpp::Time start_time, current_time;  
  config().blackboard->get("start_time", start_time);
  current_time = tf_time_;
  RCLCPP_INFO(node_->get_logger(), "[CHECK_END_TEST]: Start time: %.2f. End time: %.2f.", start_time.seconds(), current_time.seconds());
  // current_time = node_->now();
  // rclcpp::Duration duration = tf_time_ - start_time;
  float duration = current_time.seconds() - start_time.seconds();
  RCLCPP_INFO(node_->get_logger(), "[CHECK_END_TEST]: Time elapsed: %.2f seconds.", duration);
  // return duration.nanoseconds() / 1e9;
  return duration;
}

void
CheckEndTest::halt()
{
}

}  // namespace gait_speed

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<gait_speed::CheckEndTest>("CheckEndTest");
}
