from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

	ld = LaunchDescription()

	spider_bot_control_node =  Node(
			package='spider_bot',
			executable='spider_bot_control',
			name='spider_bot_control',
			output='screen',
		   )

	ld.add_action(spider_bot_control_node)

	return ld