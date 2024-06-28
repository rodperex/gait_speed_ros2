# gait_speed_ros2

Gait speed calculation using ROS 2

Modalities:

* Robot moves with the older person as they walk
  * In this case, if the robot sees an obstacle between the person and itself, it means it will be avoided, so the test will be cancelled
* Robot remains static while the older person walks at their normal speed
  * In this case, if the robot looses the person it needs to pan or move to find them
