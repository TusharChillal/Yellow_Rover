import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'rover_description'
    try:
        pkg_share = get_package_share_directory(pkg_name)
        witmotion_share = get_package_share_directory('witmotion_ros') # Find IMU package
    except Exception as e:
        print(f"Error finding package: {e}")
        return LaunchDescription()

    rsp_launch_path = os.path.join(pkg_share, 'launch', 'rsp.launch.xml')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekfConfig.yaml')
    
    imu_config_path = os.path.join(witmotion_share, 'config', 'yahboom10x.yml')

    rsp_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(rsp_launch_path)
    )


    motor_node = Node(
        package='cpp_motor',
        executable='motor_control_node',
        name='motor_control_node',
        output='screen'
    )


    imu_node = Node(
        package='witmotion_ros',
        executable='witmotion_ros_node',
        name='witmotion',
        output='screen',
        parameters=[imu_config_path],  
        remappings=[('/witmotion/imu', '/imu/data')] 
    )


    lidar_launch = None
    try:
        ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')
        lidar_launch_path = os.path.join(ydlidar_share, 'launch', 'ydlidar_launch.py')
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)
        )
    except Exception:
        pass

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',      
            'publish_tf': False,             
            'base_frame_id': 'base_footprint',    
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0,
            'laser_frame_id': 'lidar_link'   
        }]
    )


    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('/odometry/filtered', '/odom')]
    )

    launch_list = [
        rsp_launch,
        motor_node,
        imu_node,
        rf2o_node,
        TimerAction(period=2.0, actions=[ekf_node])
    ]
    
    if lidar_launch:
        launch_list.insert(2, lidar_launch)
    else:
        launch_list.append(LogInfo(msg="WARNING: YDLidar package not found. Skipping Lidar launch."))

    return LaunchDescription(launch_list)