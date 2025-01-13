# gait_speed_ros2

Gait speed calculation using ROS 2.

This repo is WORK IN PROGRESS...

## How to use it

Launch the **perception system** and activate it:
```bash
ros2 launch perception_system perception3d.launch.py # perception3d_sim_kobuki.launch.py / perception3d_sim_tiago.launch.py
ros2 lifecycle set /perception_system/perception_people_detection configure
ros2 lifecycle set /perception_system/perception_people_detection activate
```

Launch the **attention system** and activate it:
```bash
ros2 launch attention_system attention_system_base.launch.py 
```

Launch **HRI dependencies** (if using them):
```bash
ros2 launch hri_bt_nodes hri_dependencies.launch.py
```


Launch **gait speed**; either the standard approach:
```bash
ros2 launch gait_speed_ros2 gait_speed.launch.py
```
or the simplified:
```bash
ros2 launch gait_speed_ros2 gait_speed_simple.launch.py
```

## Simulated environment
We recomend running simulated experiments using the [tiago_harmonic](https://github.com/Tiago-Harmonic/tiago_harmonic.git) package. First, launch the simulator:
```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=house
```
Then, the navigation:
```bash
ros2 launch tiago_2dnav tiago_nav_bringup.launch.py is_public_sim:=True world_name:=small_house
```

## Real robot

Here you need to launch your real robot. In our case, apart from starting the robot, we also need to launch the camera:

```bash
ros2 launch oak_d_lite_camera_ros2 rgbd_stereo.launch.py use_disparity:=False use_lr_raw:=False use_pointcloud:=False
```

And a static transform:
```bash
ros2 run tf2_ros static_transform_publisher --child-frame-id oak-d-base-frame --frame-id base_footprint --x 0.0 --y 0.0 --z 1.0
```