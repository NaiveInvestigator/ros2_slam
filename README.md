## Prerequisites 
* [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
* [ROS 2 humble desktop](https://docs.ros.org/en/humble/Installation.html)
* [ORBSLAM3 Python bindings](https://github.com/PRBonn/ORB-SLAM-Python/)
* [Clearpath Simulator Prerequisites](https://docs.clearpathrobotics.com/docs/ros2humble/ros/installation/offboard_pc/)
* [Clearpath Simulator](https://docs.clearpathrobotics.com/docs/ros2humble/ros/tutorials/simulator/install)

## Useful Command List
```
ls /opt/ros/humble/share/clearpath_sensors/config
```

```
ros2 run clearpath_generator_common generate_bash -s ~/clearpath
```

```
ros2 launch clearpath_gz simulation.launch.py rviz:=true
```

```
ros2 launch orbslam3_ros2 orbslam3_ros2.launch.py camera_type:=rgbd visualize:=true
```

```
ros2 run topic_tools relay /a200_0000/sensors/camera_0/color/image /camera/rgb/image_color & ros2 run topic_tools relay /a200_0000/sensors/camera_0/depth/image /camera/depth/image
```

```
ros2 run rqt_image_view rqt_image_view /a200_0000/sensors/camera_0/color/image`
```

## Useful Resources
* [Clearpath Robot.yaml](https://docs.clearpathrobotics.com/docs/ros/config/yaml/sensors/overview)
* [Clearpath Simulator Sensor Config](https://docs.clearpathrobotics.com/docs/ros/config/yaml/sensors/overview/)
* [https://github.com/sagar16812/orbslam3_ros2 (PURE CPP CODE)](https://github.com/sagar16812/orbslam3_ros2)
