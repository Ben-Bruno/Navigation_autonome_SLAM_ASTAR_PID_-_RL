import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkg_projet = get_package_share_directory('projet_robotique_i2')
    
    params_file = os.path.join(pkg_projet, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # 1. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        # 2. Nav2 (DWA)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments=[('use_sim_time', 'true'), ('params_file', params_file)]
        ),
        # 3. SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
            launch_arguments=[('use_sim_time', 'true')]
        )
    ])