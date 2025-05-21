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
  source_frame_("map"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
    config().blackboard->get("node", node_);
    getInput("frame_name", frame_name_);

    // node_->declare_parameter("source_frame_gait_speed", "map");
    node_->get_parameter("source_frame_gait_speed", source_frame_);

}

BT::NodeStatus
StartTest::tick()
{
    RCLCPP_INFO_ONCE(node_->get_logger(), "START_TEST. Source frame: %s. Frame name: %s", source_frame_.c_str(), frame_name_.c_str());
    rclcpp::spin_some(node_->get_node_base_interface());

    try
    {
        if (!tf_buffer_.canTransform(source_frame_, frame_name_, tf2::TimePointZero)) {
            RCLCPP_INFO_ONCE(node_->get_logger(), "Waiting for transform from %s to %s", source_frame_.c_str(), frame_name_.c_str());
            return BT::NodeStatus::RUNNING;
        }
        
        auto map2start_msg = tf_buffer_.lookupTransform(source_frame_, frame_name_, tf2::TimePointZero);
        tf2::Stamped<tf2::Transform> map2start;
        tf2::fromMsg(map2start_msg, map2start);
        
        RCLCPP_INFO(node_->get_logger(), "[START_TEST]: Patient at %.2f meters from the %s frame.", map2start.getOrigin().length(), source_frame_.c_str());
        config().blackboard->set("map2start", map2start);
        config().blackboard->set("map2initial", map2start);

        config().blackboard->set("start_time", rclcpp::Time(map2start_msg.header.stamp, RCL_STEADY_TIME));

        RCLCPP_INFO(node_->get_logger(), "[START_TEST]: Setting start time to %.2f seconds", map2start_msg.header.stamp.nanosec/1e9);
        // config().blackboard->set("start_time", node_->now());

        return BT::NodeStatus::SUCCESS;
    }
    catch(const std::exception& ex)
    {
        RCLCPP_WARN(node_->get_logger(), "Start Test failed: %s", ex.what());
        return BT::NodeStatus::FAILURE;
    }
    
}

void
StartTest::halt()
{
}

}  // namespace gait_speed

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<gait_speed::StartTest>("StartTest");
}
