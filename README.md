# Eddie Ros

A ROS 2 package for control and communication with Eddie, the robot.

## Requirements

Setup the robot by following the steps in [this repo](https://github.com/Robots4Sustainability/documentation?tab=readme-ov-file#robot-setup).

## Run

Don't forget to source the workspace before running the interface:

```bash
source install/setup.bash
```

To run the eddie_ros interface, use:

```bash
ros2 run eddie_ros eddie_ros_interface --ros-args -p ethernet_if:=<eth interface> -p arm_select:=<controlled arm(s)>
```

Set the ethernet interface with the parameter `ethernet_if`. Use `ip a` to find the correct interface name. For more information on the connection to the robot, refer to the [robot setup documentation](https://github.com/Robots4Sustainability/documentation?tab=readme-ov-file#check-connection).

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


## View robot in RViz:

### 1 - Launch rviz:
```bash
ros2 launch eddie_ros view_eddie.launch.py
```

### 2a - Using real robot:
```bash
ros2 run eddie_ros eddie_ros_interface --ros-args -p ethernet_if:=<eth interface> -p arm_select:=<controlled arm(s)>
```
The robot model will snap to the pose of the real robot.

### 2b - Using simulation:
```bash
ros2 run eddie_ros sim_interface_node --ros-args -p arm_select:=<controlled arm(s)>
```
The robot model will snap to the pose of the simulation robot.

_Note: the simulation is still in development, there could be unstable behaviours._

### 3 - Test by giving commands, eg:
```bash
ros2 action send_goal right_arm/arm_control eddie_ros/action/ArmControl '{ target_pose: { position: {x: 0.2, y: 0.0, z: 0.0} } }'
```


