from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os
import yaml
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

def load_config_section(config_file, section):
    with open(config_file, 'r') as file:
        full_config = yaml.safe_load(file)
    return full_config.get(section, {})

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('robot_model'),'urdf', 'robot.urdf.xacro')
    params_file = os.path.join(get_package_share_path('robot_model'),'config', 'params.yaml')
    map_file = os.path.join(get_package_share_path('robot_model'),'map', 'map.yaml')
    bringup_launch_file = os.path.join("/opt/ros/humble/share/nav2_bringup/launch", "bringup_launch.py")

    robot_namespace = "robot_" + os.getenv('HOSTNAME')
    robot_description = ParameterValue(
    Command(['xacro ', urdf_path]),
    value_type=str
)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_namespace,
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': True},],
        remappings=[
            ('/tf', f'/{robot_namespace}/tf'),
            ('/tf_static', f'/{robot_namespace}/tf_static'),
            ('robot_description', 'visualization')
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace=robot_namespace,
        parameters=[{"use_sim_time": True}]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=robot_namespace,
        parameters=[
            {'robot_description': robot_description},
            os.path.join(get_package_share_path('robot_model'), 'config', 'ros2_control.yaml'),
            {'use_sim_time': True}
        ],
        remappings=[
            ('/tf', f'/{robot_namespace}/tf'),
            ('/tf_static', f'/{robot_namespace}/tf_static')
        ],
        output="screen"
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['diff_drive_controller'],
        namespace=robot_namespace,
        output="screen"
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state_broadcaster'],
        namespace=robot_namespace,
        output="screen"
    )

    nav2 = GroupAction([
        PushRosNamespace(robot_namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_file),
            launch_arguments={
                'use_sim_time': 'True',
                'params_file': params_file,
                'map': map_file,
                'namespace': robot_namespace
            }.items()
        )
    ])

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        controller_manager_node,
        joint_state_spawner,
        diff_drive_spawner,
        nav2
    ])