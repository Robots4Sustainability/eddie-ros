# Eddie Ros

A ROS 2 package for control and communication with Eddie, the robot.

## Requirements

Setup the robot by following the steps in [this repo](https://github.com/Robots4Sustainability/documentation?tab=readme-ov-file#robot-setup).

## Run

To run the eddie-ros interface, use

```bash
ros2 run eddie-ros eddie_ros_interface --ros-args -p ethernet_if:=<eth interface> -p arm_to_control:=<controlled arm>
```

Set the ethernet interface with the parameter `ethernet_if`. Set the controlled arm with `arm_to_control` to either `leftarm` or `rightarm`. If this parameter is omitted, the left arm will be used as default.
