import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    armbot_description = get_package_share_directory('armbot_description')
    arduinobot_description_prefix = get_package_prefix('armbot_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(armbot_description, 'urdf', 'armbot.urdf.xacro'),
        description='Absolute path to robot urdf file')

    model_path = os.path.join(armbot_description, "models")
    model_path += os.pathsep + os.path.join(arduinobot_description_prefix, "share")

    env_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        )
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                       arguments=['-entity', 'armbot',
                                  '-topic', 'robot_description',
                                  ],
                       output='screen'
                       )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(armbot_description, 'rviz', 'display.rviz')],
    )

    return LaunchDescription([
        env_var,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        joint_state_publisher_gui_node,
        rviz_node
    ])
