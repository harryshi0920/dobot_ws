import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 启动ros2 launch dobot_moveit dobot_moveit.launch.py
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dobot_moveit'),
                'launch',
                'dobot_moveit.launch.py'
            )
        )
    )

    scene_loader = Node(
        package='dobot_planning_scene',
        executable='add_scene',
        name='planning_scene_loader',
        output='screen',
        parameters=[
            {'frame_id': 'base_link'}
        ]
    )

    return LaunchDescription([
        moveit_launch,
        scene_loader
    ])