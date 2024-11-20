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


#include "gait_speed_ros2/behaviors/behavior_runner.hpp"

namespace gait_speed
{

BehaviorRunner::BehaviorRunner(
  BT::Blackboard::Ptr blackboard,
  const std::string &name,
  const std::string &xml_path,
  const std::vector<std::string> &plugins)
  : CascadeLifecycleNode(name),
  blackboard_(blackboard)
{
  RCLCPP_INFO(get_logger(), "BehaviorRunner constructor (%s)", name.c_str());
  
  blackboard_->get("node", node_);

  xml_path_ = xml_path;
  plugins_ = plugins;

  RCLCPP_INFO(get_logger(), "XML path: %s", xml_path_.c_str());
  RCLCPP_INFO(get_logger(), "# plugins: %d", plugins_.size());

  status_pub_ = create_publisher<std_msgs::msg::String>("behavior_status", 10);
}

void
BehaviorRunner::control_cycle()
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
      RCLCPP_DEBUG(get_logger(), "Behavior tree is running");
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
BehaviorRunner::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "BehaviorRunner (%s) on_activate", get_name());

  std::string pkg_path = ament_index_cpp::get_package_share_directory("gait_speed_ros2");
  std::string xml_file = pkg_path + xml_path_;

  RCLCPP_DEBUG(get_logger(), "XML file: %s", xml_file.c_str());

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  for (const auto & plugin : plugins_) {
    factory.registerFromPlugin(loader.getOSName(plugin));
  }

  tree_ = factory.createTreeFromFile(xml_file, blackboard_);

  status_pub_->on_activate();

  timer_ =
    create_wall_timer(50ms, std::bind(&BehaviorRunner::control_cycle, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BehaviorRunner::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "BehaviorRunner(%s) on_deactivate", get_name());

  timer_ = nullptr;
  status_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace gait_speed
