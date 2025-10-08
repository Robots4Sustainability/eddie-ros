# Eddie Ros

A ROS 2 package for control and communication with Eddie, the robot.

## Requirements

Setup the robot by following the steps in [this file](robot_setup.md).

## Run

Don't forget to source the workspace before running the interface:

```bash
source install/setup.bash
```

To run the eddie_ros interface, use:

```bash
ros2 run eddie_ros eddie_ros_interface --ros-args -p ethernet_if:=<eth interface> -p arm_select:=<controlled arm(s)>
```

Set the ethernet interface with the parameter `ethernet_if`. Use `ip a` to find the correct interface name. For more information on the connection to the robot, refer to the [robot setup documentation](robot_setup.md#check-connection).

Set the controlled arm(s) with `arm_select` to either `left`, `right` or `both`. This parameter is required.

## ROS2 Actions

You can send goals to the robot using ROS2 actions depending on the selected arm(s).

- `left_arm/arm_control` and `right_arm/arm_control` of type `eddie_ros/action/ArmControl` for controlling the arm's target pose.
- `left_arm/gripper_control` and `right_arm/gripper_control` of type `eddie_ros/action/GripperControl` for controlling the gripper's position, velocity, and force.

For example, to move the right arm to a target pose (here, 10 cm in the z direction):

```bash
ros2 action send_goal right_arm/arm_control eddie_ros/action/ArmControl '{ target_pose: { position: {x: 0.0, y: 0.0, z: 0.1} } }'
```

To control the gripper of the right arm:

```bash
ros2 action send_goal right_arm/gripper_control eddie_ros/action/GripperControl '{ target_position: 50.0, velocity: 20.0, force: 20.0 }'
```

Check out the [action definitions](action) for more details on how to define goals.

![Eddie axis reference frame](ee_axis.png)

The image above shows the axis reference frame for the Kinova Manipulator end-effector coordinate system. Also see the [manual](https://github.com/Robots4Sustainability/documentation/blob/main/manuals/EN-UG-014-Gen3-Ultra-lightweight-user-guide-r10.0.pdf) for more information.
