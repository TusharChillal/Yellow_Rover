import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Change 'rover_description' to your actual package name where you saved config/ekf.yaml
    pkg_name = 'rover_description' 
    
    # Path to the config file
    # Ensure ekfConfig.yaml is inside the 'config' folder of your package
    try:
        pkg_share = get_package_share_directory(pkg_name)
        # UPDATED: Matches your actual filename 'ekfConfig.yaml'
        ekf_config_path = os.path.join(pkg_share, 'config', 'ekfConfig.yaml')
    except Exception:
        # Fallback if package is not built yet (for testing)
        ekf_config_path = 'config/ekfConfig.yaml'

    return LaunchDescription([
        # Robot Localization Node (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            # Optional: Remap output if you want it to be just /odom
            # This ensures the EKF output is published to /odom, which is what Nav2 expects
            remappings=[('/odometry/filtered', '/odom')] 
        ),
    ])