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
    config().blackboard->get("mode", mode_);
}

BT::NodeStatus
EndTest::tick()
{
    if (mode_ == "distance") {
        float time_elapsed;
        config().blackboard->get("time_elapsed", time_elapsed);
        
        if (time_elapsed > 0.0) {
            RCLCPP_INFO(node_->get_logger(), "Test finished. Time: %f seconds", time_elapsed);
            config().blackboard->set("gait_speed_result", time_elapsed);
            return BT::NodeStatus::SUCCESS;
        }
    } else {
        float distance_travelled;
        config().blackboard->get("distance_travelled", distance_travelled);

        if (distance_travelled > 0) {
            RCLCPP_INFO(node_->get_logger(), "Test finished. Distance: %f meters", distance_travelled);
            config().blackboard->set("gait_speed_result", distance_travelled);
            return BT::NodeStatus::SUCCESS;
        }
    }
    RCLCPP_WARN(node_->get_logger(), "Gait speed test finished with error");
    return BT::NodeStatus::FAILURE;
}

void
EndTest::halt()
{
}

}  // namespace gait_speed

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gait_speed::EndTest>("EndTest");
}