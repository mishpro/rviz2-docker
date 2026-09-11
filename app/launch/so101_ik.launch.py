import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # По умолчанию ищем URDF по этому пути
    default_urdf = '/workspace/SO-ARM100/Simulation/SO101/so101_new_calib.urdf'
    default_rviz = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'so101.rviz')

    urdf_path = LaunchConfiguration('urdf_path')
    rviz_config = LaunchConfiguration('rviz_config')

    # Читаем URDF в строку для robot_state_publisher
    with open(default_urdf, 'r') as f:
        urdf_content = f.read().replace(
            'filename="assets/',
            'filename="file:///workspace/SO-ARM100/Simulation/SO101/assets/')

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_path',
            default_value=default_urdf,
            description='Path to SO-101 URDF file'),

        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='Path to RViz config file'),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_content}],
            output='screen',
        ),

        # IK node
        Node(
            package='so101_ik',
            executable='ik_example',
            name='so101_ik_node',
            arguments=[default_urdf],
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz],
            output='screen',
        ),
    ])
