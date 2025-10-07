import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():

    arm_select_arg = DeclareLaunchArgument(
        "arm_select", default_value="both", description="Control 'left', 'right', or 'both' arms"
    )

    # Process the XACRO file to get the robot_description string
    eddie_description_pkg = get_package_share_directory("eddie_description")
    xacro_file = os.path.join(eddie_description_pkg, "urdf", "eddie_robot.urdf.xacro")
    
    # For the real robot, don't use fake hardware
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ", xacro_file,
            " use_fake_hardware:=false",
            " use_ros2_control:=false",
        ]
    )
    robot_description_param = {"robot_description": robot_description_content}

    # Nodes to Launch

    # pass the processed robot_description as a parameter.
    robot_interface_node = Node(
        package="eddie_ros",
        executable="eddie_ros_interface",
        name="eddie_ros_interface",
        output="screen",
        parameters=[
            robot_description_param,
            {"arm_select": LaunchConfiguration("arm_select")}
        ]
    )

    # Reuses the display launch file but disables its fake joint publisher.
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(eddie_description_pkg, "launch", "display_eddie.launch.py")
        ),
        launch_arguments={
            "joint_state_gui": "false"
        }.items()
    )

    return LaunchDescription([
        arm_select_arg,
        display_launch,
        robot_interface_node,
    ])