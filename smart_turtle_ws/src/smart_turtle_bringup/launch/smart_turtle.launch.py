from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


import os


def generate_launch_description():
    ld = LaunchDescription()

    param_config = os.path.join(
        get_package_share_directory('smart_turtle_bringup'),
        'config',
        'turtle_param.yaml'
    )


    publisher = Node(
        package='smart_turtle_pkg',
        executable='command_publisher',
        name='command_publisher',


        parameters=[param_config]
    )


    controller = Node(
        package='smart_turtle_pkg',
        executable='turtle_controller',
        name='turtle_controller'
    )

    reset = Node(
        package='smart_turtle_pkg',
        executable='reset_node',
        name='reset_node'
    )

    turtle = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtle'
    )

    ld.add_action(publisher)
    ld.add_action(controller)
    ld.add_action(reset)
    ld.add_action(turtle)


    return ld
    