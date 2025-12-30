import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_lumen = get_package_share_directory('lumen_bringup')
    
    # Path to your saved map
    map_file = os.path.join(pkg_lumen, 'maps', 'map.yaml')
    nav2_params = os.path.join(pkg_lumen, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # 1. Start your Robot Driver + EKF (Same as before)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_lumen, 'launch', 'bringup.launch.py')
            )
        ),

        # 2. Start Nav2 (Localization + Path Planning)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'params_file': nav2_params,
                'use_sim_time': 'false'
            }.items()
        ),
       
    ])
