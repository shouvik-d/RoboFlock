import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robo_sims')
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file)
    robot_urdf = robot_description.toxml()
    
    rviz_config = os.path.join(pkg_share, 'config', 'urdf.rviz')

    gui_arg = DeclareLaunchArgument(
        name = 'gui',
        default_value = 'true',
    )

    show_gui = LaunchConfiguration('gui')

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    return LaunchDescription([
        gui_arg,

        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_urdf}]
        ),
        
        joint_state_publisher_node,

        Node(
            condition=IfCondition(show_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
