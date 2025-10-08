import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():

    # Decide whether to visualize a sim or real robot to load the URDF correctly
    use_sim_arg = DeclareLaunchArgument(
        "use_sim", default_value="false",
        description="Set to 'true' if visualizing the simulation."
    )

    eddie_description_pkg = get_package_share_directory("eddie_description")
    
    # Process the XACRO file to get the robot_description
    xacro_file = os.path.join(eddie_description_pkg, "urdf", "eddie_robot.urdf.xacro")
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file,
        " use_ros2_control:=false",
        " use_fake_hardware:=", LaunchConfiguration("use_sim"),
    ])
    robot_description_param = {"robot_description": robot_description_content}

    # Node to publish the robot's static transformations (TF tree)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param]
    )
    
    # RViz2 node
    rviz_config_file = os.path.join(eddie_description_pkg, "config/rviz", "eddie.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        use_sim_arg,
        robot_state_publisher_node,
        rviz_node,
    ])