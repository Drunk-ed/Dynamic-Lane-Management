from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    model = os.environ["TURTLEBOT3_MODEL"]

    sdf = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "models",
        "turtlebot3_" + model,
        "model.sdf",
    )

    return LaunchDescription([

        DeclareLaunchArgument("entity"),
        DeclareLaunchArgument("x"),
        DeclareLaunchArgument("y"),
        DeclareLaunchArgument("z", default_value="0.01"),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=[
                "-entity",
                LaunchConfiguration("entity"),

                "-robot_namespace",
                LaunchConfiguration("entity"),

                "-file",
                sdf,

                "-x",
                LaunchConfiguration("x"),

                "-y",
                LaunchConfiguration("y"),

                "-z",
                LaunchConfiguration("z"),
            ],
        ),
    ])
