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


#include "gait_speed_ros2/bt_nodes/isMyPerson.hpp"

namespace gait_speed
{
IsMyPerson::IsMyPerson(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  threshold_(0.5)
{
  getInput("person_id", person_id_);
  config().blackboard->get("node", node_);
}

BT::NodeStatus
IsMyPerson::tick()
{
  pl::getInstance(node_)->set_interest("person", true);
  pl::getInstance(node_)->update(30);
  pl::getInstance(node_)->publicTFinterest();

  std::vector<perception_system_interfaces::msg::Detection> detections;
  detections = pl::getInstance(node_)->get_by_type("person");

  if (detections.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No detections");
    return BT::NodeStatus::FAILURE;
  }

  auto best_detection = detections[0];

  if (perception_system::diffIDs(person_id_, best_detection.color_person) > threshold_) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace gait_speed
