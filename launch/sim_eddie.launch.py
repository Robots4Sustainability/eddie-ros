import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():

    arm_select_arg = DeclareLaunchArgument(
        "arm_select", default_value="both"
    )

    eddie_description_pkg = get_package_share_directory("eddie_description")
    eddie_ros_pkg = get_package_share_directory("eddie_ros")

    # Process the XACRO file to get the robot_description string
    xacro_file = os.path.join(eddie_description_pkg, "urdf", "eddie_robot.urdf.xacro")
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ", xacro_file,
            " use_fake_hardware:=true",
            " use_ros2_control:=false",
        ]
    )
    robot_description_param = {"robot_description": robot_description_content}

    # Nodes to launch

    # simulates the robot's hardware and publishes the /joint_states.
    sim_interface_node = Node(
        package="eddie_ros",
        executable="sim_interface_node",
        name="sim_interface_node",
        output="screen",
        parameters=[
            robot_description_param,
            {"arm_select": LaunchConfiguration("arm_select")}
        ]
    )
    
    # reads /joint_states and the robot_description to publish the TF tree for RViz.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param]
    )
    
    # find the official RViz config file from the eddie_description package.
    rviz_config_file = os.path.join(eddie_description_pkg, "config/rviz", "eddie.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        arm_select_arg,
        robot_state_publisher_node,
        rviz_node,
        sim_interface_node,
    ])