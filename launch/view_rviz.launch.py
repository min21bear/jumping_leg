#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find the package directory
    pkg_path = os.path.join('/home/road2022/mingue/src', 'jumping_robot')
    
    # Set the path to the URDF file
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    
    # Set the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_path, 'config', 'view_robot.rviz')
    
    # Create a launch argument for enabling/disabling the GUI joint state publisher
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )

    with open(urdf_file, 'r') as f:
        robot_description_config = f.read()

    robot_description = {'robot_description': robot_description_config}
    
    # Start the robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )
    
    # Start the joint state publisher (non-GUI version)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('gui', default='false'))
    )
    
    # Start the joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui', default='true'))
    )
    
    # Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])