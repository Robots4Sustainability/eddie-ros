# Eddie Ros

A ROS 2 package for control and communication with Eddie, the robot.

## Requirements

Setup the robot by following the steps in [this repo](https://github.com/Robots4Sustainability/documentation?tab=readme-ov-file#robot-setup).

## Run

Don't forget to source the workspace before running the interface:

```bash
source install/setup.bash
```

To run the eddie-ros interface, use:

```bash
ros2 run eddie-ros eddie_ros_interface --ros-args -p ethernet_if:=<eth interface> -p arm_select:=<controlled arm(s)>
```

Set the ethernet interface with the parameter `ethernet_if`. Use `ip a` to find the correct interface name. For more information on the connection to the robot, refer to the [robot setup documentation](https://github.com/Robots4Sustainability/documentation?tab=readme-ov-file#check-connection).

Set the controlled arm(s) with `arm_select` to either `left`, `right` or `both`. This parameter is required.
