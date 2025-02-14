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
    getInput("mode", mode_);
    result_pub_ = node_->create_publisher<std_msgs::msg::Float64>("gait_speed_result", 1);
    result_pub_->on_activate();
}

BT::NodeStatus
EndTest::tick()
{
    RCLCPP_INFO(node_->get_logger(), "END_TEST");
    
    if (mode_ == "distance") {
        float time_elapsed;
        config().blackboard->get("time_elapsed", time_elapsed);
        
        if (time_elapsed > 0.0) {
            RCLCPP_INFO(node_->get_logger(), "[END_TEST]: Test finished. Time: %.2f seconds", time_elapsed);
            config().blackboard->set("gait_speed_result", time_elapsed);
            setOutput("result", time_elapsed);
            std_msgs::msg::Float64 msg;
            msg.data = time_elapsed;
            result_pub_->publish(msg);
            return BT::NodeStatus::SUCCESS;
        }
    } else {
        float distance_travelled;
        config().blackboard->get("distance_travelled", distance_travelled);

        if (distance_travelled > 0) {
            RCLCPP_INFO(node_->get_logger(), "[END_TEST]: Test finished. Distance: %f meters", distance_travelled);
            config().blackboard->set("gait_speed_result", distance_travelled);
            setOutput("result", distance_travelled);
            std_msgs::msg::Float64 msg;
            msg.data = distance_travelled;
            result_pub_->publish(msg);
            return BT::NodeStatus::SUCCESS;
        }
    }
    config().blackboard->set("gait_speed_result", -1.0);
    RCLCPP_WARN(node_->get_logger(), "[END_TEST]: Gait speed test finished with error");
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