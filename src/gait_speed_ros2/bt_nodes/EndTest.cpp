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


#include "gait_speed_ros2/bt_nodes/EndTest.hpp"

namespace gait_speed
{
EndTest::EndTest(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
    config().blackboard->get("node", node_);
}

BT::NodeStatus
EndTest::tick()
{
    rclcpp::Time start_time;
    config().blackboard->get("start_time", start_time);

    rclcpp::Duration duration = node_->now() - start_time;
    
    float seconds = duration.nanoseconds() / 1e9;

    if (seconds > 0) {
        RCLCPP_INFO(node_->get_logger(), "Test finished. Time: %f", seconds);
        config().blackboard->set("gait_speed_result", seconds);
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_WARN(node_->get_logger(), "Test finished with error");
        return BT::NodeStatus::FAILURE;
    }

}

}  // namespace gait_speed
