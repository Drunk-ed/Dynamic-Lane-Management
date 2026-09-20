from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")

    multi_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                tb3_gazebo,
                "launch",
                "multi_robot.launch.py"
            )
        )
    )

    return LaunchDescription([
        multi_robot
    ])
