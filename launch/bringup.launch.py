import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'lumen_bringup'
    
    # Paths
    ekf_config_path = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf.yaml')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # ========================================================================
    # 1. RPLIDAR Driver (A1M8)
    # ========================================================================
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,      # Integer, not string
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'         
        }]
    )

    # ========================================================================
    # 2. Static Transform (base_link -> laser)
    # ========================================================================
    # Adjust args: x y z yaw pitch roll parent child
    tf_laser = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_laser',
    arguments=[
        '--x', '0.2',
        '--y', '0.0',
        '--z', '0.0',
        '--roll', '0.0',
        '--pitch', '0.0',
        '--yaw', '0.0',
        '--frame-id', 'base_link',
        '--child-frame-id', 'laser'
    ])

    # ========================================================================
    # 3. Robot Localization (EKF)
    # ========================================================================
    #ekf_node = Node(
    #    package='robot_localization',
    #    executable='ekf_node',
    #    name='ekf_filter_node',
    #    output='screen',
    #    parameters=[ekf_config_path]
    #)

    # ========================================================================
    # 4. SLAM Toolbox (Mapping)
    # ========================================================================
    #slam_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    #    ),
    #    launch_arguments={'use_sim_time': 'false'}.items()
    #)

    return LaunchDescription([
        rplidar_node,  # <--- Added here
        tf_laser,
        #ekf_node,
        #slam_launch
    ])
