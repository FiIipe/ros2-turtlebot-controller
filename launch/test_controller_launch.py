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
        package='',
        executable='',
        name='',
        output='screen',
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtlebot_controller_node)

    return ld
