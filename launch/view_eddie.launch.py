import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Path to the eddie_ros package
    pkg_path = get_package_share_directory('eddie_ros')

    # Path to the URDF file
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'eddie.urdf')

    # Load the URDF content
    robot_description_config = xacro.process_file(urdf_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Nodes to Launch

    # 1. Robot State Publisher
    # Reads the URDF and publishes the robot's static transformations (tf)
    # based on the /joint_states topic.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. RViz2
    # Provides the 3D visualization
    viz_config_file = os.path.join(pkg_path, 'config', 'eddie.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', viz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
