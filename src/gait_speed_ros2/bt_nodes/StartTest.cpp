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


#include "gait_speed_ros2/bt_nodes/StartTest.hpp"

namespace gait_speed
{
StartTest::StartTest(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  distance_(0.0),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
    getInput("distance", distance_);
    config().blackboard->get("node", node_);
}

BT::NodeStatus
StartTest::tick()
{
    RCLCPP_INFO(node_->get_logger(), "Starting test");

    try
    {
        auto odom2bf0_msg = tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
        tf2::Stamped<tf2::Transform> odom2bf0;
        tf2::fromMsg(odom2bf0_msg, odom2bf0);

        rclcpp::Time start_time = node_->now();
        
        config().blackboard->set("start_time", start_time);
        config().blackboard->set("distance", distance_);
        config().blackboard->set("odom2bf0", odom2bf0);
    
        return BT::NodeStatus::SUCCESS;
    }
    catch(const std::exception& ex)
    {
        RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
        return BT::NodeStatus::FAILURE;
    }
    
}

}  // namespace gait_speed
