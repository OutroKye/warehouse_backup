# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
	# Get the launch directory
	aws_small_warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')

	# Launch configuration variables specific to simulation
	use_sim_time = LaunchConfiguration('use_sim_time')
	use_simulator = LaunchConfiguration('use_simulator')
	headless = LaunchConfiguration('headless')
	world = LaunchConfiguration('world')

	declare_use_sim_time_cmd = DeclareLaunchArgument(
		'use_sim_time',
		default_value='True',
		description='Use simulation (Gazebo) clock if true')

	declare_simulator_cmd = DeclareLaunchArgument(
		'headless',
		default_value='False',
		description='Whether to execute gzclient)')

	declare_world_cmd = DeclareLaunchArgument(
		'world',
		default_value=os.path.join(aws_small_warehouse_dir, 'worlds', '2_floor_warehouse.world'),
		description='Full path to world model file to load')

	# Specify the actions
	start_gazebo_server_cmd = ExecuteProcess(
		cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
		cwd=[aws_small_warehouse_dir], output='screen')

	start_gazebo_client_cmd = ExecuteProcess(
		condition=IfCondition(PythonExpression(['not ', headless])),
		cmd=['gzclient'],
		cwd=[aws_small_warehouse_dir], output='screen')

	# Create the launch description and populate
	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(declare_simulator_cmd)
	ld.add_action(declare_world_cmd)

	# Add any conditioned actions
	ld.add_action(start_gazebo_server_cmd)
	ld.add_action(start_gazebo_client_cmd)

	return ld
