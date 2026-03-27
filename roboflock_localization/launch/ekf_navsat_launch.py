from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    roboflock_localization_dir = get_package_share_directory('roboflock_localization')
    parameters_file_dir = os.path.join(roboflock_localization_dir, 'params')
    parameters_file_path = os.path.join(parameters_file_dir, 'ekf_navsat_params.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
	    default_value='~/ekf_navsat_debug.txt'),
	
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odometry/local')]           
           ),
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odometry/global')]
           ),           
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('imu', 'imu/data'),
                        ('gps/fix', 'gps/fix'), 
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/local')]           

           ),
    launch_ros.actions.Node(
            package='nmea_navsat_driver', 
            executable='nmea_serial_driver', 
            name='gps',
            output='screen',
            parameters=[os.path.join(parameters_file_dir, 'gps.yaml')],
            remappings=[('sensor_msgs/NavSatFix', 'sensor_msgs/NavSatFix')]
    ),
    launch_ros.actions.Node(
            package='mpu9250driver', 
            executable='mpu9250driver', 
            name='imu',
            output='screen',
            parameters=[os.path.join(parameters_file_dir, 'mpu9250.yaml')],
            remappings=[('sensor_msgs/Imu', 'sensor_msgs/Imu')]
    )                
])
