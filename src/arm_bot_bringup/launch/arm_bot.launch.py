import os
from sys import executable

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("armbot_description"),
            "launch",
            "gazebo.launch.py"
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arm_bot_bringup"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={"is_sim": "False"}.items()
    )

    tcp_server_bridge_node = Node(
        package="server_bridge",
        executable="tcp_server",
        name="tcp_server"
    )

    websocket_server_bridge_node = Node(
        package="server_bridge",
        executable="websocket_server",
        name="websocket_server"
    )

    mqtt_interface_node = Node(
        package="arm_bot_mqtt_interface",
        executable="arm_bot_mqtt_interface",
        name="arm_bot_mqtt_interface"
    )

    return LaunchDescription([
        certificate_arg,
        private_key_arg,
        gazebo,
        controller,
        tcp_server_bridge_node,
        websocket_server_bridge_node,
        mqtt_interface_node
    ])
