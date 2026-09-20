#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool, HyunGyu Kim

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def cleanup_tmp_files(save_path, robots):
	for robot in robots:
		tmp_file = os.path.join(save_path, f'{robot["name"]}.sdf')
		if os.path.exists(tmp_file):
			os.remove(tmp_file)


def generate_launch_description():
	TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

	robots = [
		{
			"name": "divider_01",
			"namespace": "divider_01",
			"x": 0.0,
			"y": 0.0,
		},
		{
			"name": "divider_02",
			"namespace": "divider_02",
			"x": 3.0,
			"y": 0.0,
		},
	]
	model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
	urdf_path = os.path.join(
		get_package_share_directory('turtlebot3_gazebo'),
	'models',
	model_folder,
		'model.sdf'
	)
	save_path = os.path.expanduser("~/.lane_bringup/tmp/")
	os.makedirs(save_path, exist_ok=True)
	launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')


	use_sim_time = LaunchConfiguration('use_sim_time', default='true')



  

	robot_state_publisher_cmd_list = []

	for robot in robots:
		robot_state_publisher_cmd_list.append(
			IncludeLaunchDescription(
				PythonLaunchDescriptionSource(
					os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
				),
				launch_arguments={
					'use_sim_time': use_sim_time,
					'frame_prefix': robot["namespace"]
					}.items()
			)
		)

	spawn_turtlebot_cmd_list = []

	for robot in robots:
		tree = ET.parse(urdf_path)
		root = tree.getroot()
		for odom_frame_tag in root.iter('odometry_frame'):
			odom_frame_tag.text = f'{robot["namespace"]}/odom'
		for base_frame_tag in root.iter('robot_base_frame'):
			base_frame_tag.text = f'{robot["namespace"]}/base_footprint'
		for scan_frame_tag in root.iter('frame_name'):
			scan_frame_tag.text = f'{robot["namespace"]}/base_scan'
		urdf_modified = ET.tostring(tree.getroot(), encoding='unicode')
		urdf_modified = '<?xml version="1.0" ?>\n'+urdf_modified
		tmp_file = os.path.join(
			save_path,
			f'{robot["name"]}.sdf'
		)

		with open(tmp_file, "w") as file:
			file.write(urdf_modified)
		spawn_turtlebot_cmd_list.append(
			IncludeLaunchDescription(
				PythonLaunchDescriptionSource(
					os.path.join(launch_file_dir, 'multi_spawn_turtlebot3.launch.py')
				),
				launch_arguments={
						'x_pose': str(robot["x"]),
						'y_pose': str(robot["y"]),
						'robot_name': robot["name"],
						'namespace': robot["namespace"],
						'sdf_path': tmp_file
				}.items()
			)
		)

	ld = LaunchDescription()
	# Add the commands to the launch description
	ld.add_action(
		RegisterEventHandler(
			OnShutdown(
				on_shutdown=lambda event, context:
					cleanup_tmp_files(save_path, robots)
			)
		)
	)
	for i, robot in enumerate(robots):

		ld.add_action(

			GroupAction([
			
				PushRosNamespace(robot["namespace"]),

				robot_state_publisher_cmd_list[i],

				spawn_turtlebot_cmd_list[i],
			])
		)

	return ld
