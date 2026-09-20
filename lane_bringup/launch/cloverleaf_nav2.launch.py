from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def nav2(namespace, params):

    return IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(
                get_package_share_directory("lane_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),

        launch_arguments={

            "namespace": namespace,
            "use_namespace": "True",
            "slam": "False",

            "map":
            os.path.join(
                get_package_share_directory("turtlebot3_navigation2"),
                "map",
                "map.yaml",
            ),

            "params_file": params,
            
            "use_composition": "False",

            "use_sim_time": "True",

            "autostart": "True",

        }.items(),

    )


def generate_launch_description():

    return LaunchDescription([

        nav2(

            "divider_01",

            os.path.join(

                get_package_share_directory("lane_bringup"),
                "param",
                "nav2_multirobot.yaml",

            ),

        ),

    ])
