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


#include "gait_speed_ros2/behaviors/measure_gait_speed_dist.hpp"

namespace gait_speed
{

MeasureGaitSpeedDist::MeasureGaitSpeedDist(BT::Blackboard::Ptr blackboard)
: CascadeLifecycleNode("measure_gait_speed_dist")
{
  RCLCPP_INFO(get_logger(), "MeasureGaitSpeedDist constructor");

  std::string pkg_path = ament_index_cpp::get_package_share_directory("gait_speed_ros2");
  std::string xml_file = pkg_path + "/bt_xml/measure_gait_speed_dist.xml";
  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("is_detected_bt_node"));
  factory.registerFromPlugin(loader.getOSName("is_my_person_bt_node"));
  factory.registerFromPlugin(loader.getOSName("start_test_bt_node"));
  factory.registerFromPlugin(loader.getOSName("end_test_bt_node"));

  blackboard_ = blackboard;
  blackboard_->get<rclcpp::Node::SharedPtr>("node", node_);

  tree_ = factory.createTreeFromFile(xml_file, blackboard);

  status_pub_ = create_publisher<std_msgs::msg::String>("behavior_status", 10);
}

void
MeasureGaitSpeedDist::control_cycle()
{
  std_msgs::msg::String msg;
  BT::NodeStatus status = tree_.rootNode()->executeTick();

  switch (status) {
    case BT::NodeStatus::SUCCESS:
      RCLCPP_INFO(get_logger(), "Behavior tree executed successfully");
      msg.data = "SUCCESS";
      status_pub_->publish(msg);
      break;
    case BT::NodeStatus::RUNNING:
      RCLCPP_INFO(get_logger(), "Behavior tree is running");
      msg.data = "RUNNING";
      status_pub_->publish(msg);
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Behavior tree failed");
      msg.data = "FAILURE";
      status_pub_->publish(msg);
      break;
  }

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MeasureGaitSpeedDist::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "MeasureMGaitSpeedDist on_activate");

  status_pub_->on_activate();

  timer_ =
    create_wall_timer(50ms, std::bind(&MeasureGaitSpeedDist::control_cycle, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MeasureGaitSpeedDist::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "MeasureMGaitSpeedDist on_deactivate");

  timer_ = nullptr;
  status_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace gait_speed
