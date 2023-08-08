from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
    )

    turtlebot_controller_node = Node(
        package='turtlebot_control',
        executable='basic_controller_exe',
        name='basic_controller_node',
        output='screen',
        remappings=[('cmd_vel', 'turtle1/cmd_vel')],
        parameters=[
            {"linear_velocity": 1.0},
            {"angular_velocity": 1.0},
        ]
    )

    turtlebot_controller_service_client_node = Node(
        package='turtlebot_control',
        executable='basic_controller_service_client_exe',
        name='basic_controller_service_client_node',
        output='screen',
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtlebot_controller_node)
    ld.add_action(turtlebot_controller_service_client_node)

    return ld
