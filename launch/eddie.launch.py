import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    # Declare all launch arguments
    use_sim_arg = DeclareLaunchArgument(
        "use_sim", default_value="false",
        description="Set to 'true' to use simulation, 'false' for real robot."
    )
    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz", default_value="false",
        description="Set to 'true' to also launch RViz on startup."
    )
    arm_select_arg = DeclareLaunchArgument(
        "arm_select",
        description="Control 'left', 'right', or 'both' arms."
    )
    ethernet_if_arg = DeclareLaunchArgument(
        "ethernet_if", default_value="eth0",
        description="Ethernet interface for the real robot."
    )
    ft_sensor_com_port_arg = DeclareLaunchArgument(
        "ft_sensor_com_port", default_value="",
        description="COM port for the FT sensor (e.g., '/dev/ttyUSB0')."
    )

    # Process the XACRO file to get the robot_description
    eddie_description_pkg = get_package_share_directory("eddie_description")
    xacro_file = os.path.join(eddie_description_pkg, "urdf", "eddie_robot.urdf.xacro")
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file,
        " use_ros2_control:=false",
        " use_fake_hardware:=", LaunchConfiguration("use_sim"),
    ])
    robot_description_param = {"robot_description": robot_description_content}

    # Simulation Node
    simulation_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_sim")),
        actions=[
            Node(
                package="eddie_ros",
                executable="sim_interface_node",
                name="sim_interface_node",
                output="screen",
                parameters=[
                    robot_description_param,
                    {"arm_select": LaunchConfiguration("arm_select")}
                ]
            )
        ]
    )

    # Real Robot Node
    real_robot_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration("use_sim")),
        actions=[
            Node(
                package="eddie_ros",
                executable="eddie_ros_interface",
                name="eddie_ros_interface",
                output="screen",
                parameters=[
                    robot_description_param,
                    {"arm_select": LaunchConfiguration("arm_select")},
                    {"ethernet_if": LaunchConfiguration("ethernet_if")},
                    {"ft_sensor_com_port": LaunchConfiguration("ft_sensor_com_port")},
                ]
            )
        ]
    )

    # Publish joint states
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param]
    )

    # RViz2 node
    rviz_config_file = os.path.join(eddie_description_pkg, "config/rviz", "eddie.rviz")
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("show_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file]
    )
    
    return LaunchDescription([
        use_sim_arg,
        show_rviz_arg,
        arm_select_arg,
        ethernet_if_arg,
        ft_sensor_com_port_arg,

        simulation_group,
        real_robot_group,
        robot_state_publisher_node,
        rviz_node,
    ])
