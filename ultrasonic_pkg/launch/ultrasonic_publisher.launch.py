#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import node

def generate_launch_description():
	declare_params_file = DeclareLaunchArgument(
		'params_file',
		default_value='',
		description='Path to YAML parameters file'
	)
	
	declare_use_sim_time = DeclareLaunchArgument(
		'use_sim_time',
		default_value='false',
		description='Use simulation time'
	)
	
	ultrasonic_publisher = Node(
		package='ultrasonic_pkg',
		executable='ultrasonic_publisher',
		name='ultrasonic_publisher',
		output='screen',
		parameters=[
			LaunchConfiguration('params_file'),
			{
				'use_sim_time': LaunchConfiguration('use_sim_time')
			}
		]
	)
	
	return LaunchDescription([
		declare_params_file,
		declare_use_sim_time,
		ultrasonic_publisher
	])
	
	
	
