import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'rover_description' 
    
    try:
        pkg_share = get_package_share_directory(pkg_name)
        ekf_config_path = os.path.join(pkg_share, 'config', 'ekfConfig.yaml')
    except Exception:
        ekf_config_path = 'config/ekfConfig.yaml'

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[('/odometry/filtered', '/odom')] 
        ),
    ])
