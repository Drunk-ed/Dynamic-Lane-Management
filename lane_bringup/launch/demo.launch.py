from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    lane_pkg = get_package_share_directory("lane_bringup")

    return LaunchDescription([

        ##################################################
        # Gazebo + Robots
        ##################################################

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    lane_pkg,
                    "launch",
                    "two_robot.launch.py"
                )
            )
        ),

        ##################################################
        # Nav2
        ##################################################

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    lane_pkg,
                    "launch",
                    "two_robot_nav.launch.py"
                )
            )
        ),

        ##################################################
        # Occupancy Detector TB3_1
        ##################################################

        Node(
            package="road_occupancy",
            executable="occupancy_detector",
            name="occupancy_tb3_1",
            parameters=[
                {"robot_namespace": "TB3_1"}
            ],
            output="screen",
        ),

        ##################################################
        # Occupancy Detector TB3_2
        ##################################################

        Node(
            package="road_occupancy",
            executable="occupancy_detector",
            name="occupancy_tb3_2",
            parameters=[
                {"robot_namespace": "TB3_2"}
            ],
            output="screen",
        ),

        ##################################################
        # Fleet Coordinator
        ##################################################

        Node(
            package="fleet_coordinator",
            executable="coordinator",
            output="screen",
        ),
    ])
