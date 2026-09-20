#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([

        DeclareLaunchArgument(
            "namespace",
            default_value=""
        ),

        DeclareLaunchArgument(
            "params_file"
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            namespace=namespace,
            output="screen",
            parameters=[params_file],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
                ("cmd_vel", "cmd_vel_nav"),
            ]
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            namespace=namespace,
            output="screen",
            parameters=[params_file],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ]
        ),

        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            namespace=namespace,
            output="screen",
            parameters=[params_file],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel"),
            ]
        ),

       # Node(
        #    package="nav2_lifecycle_manager",
         #   executable="lifecycle_manager",
          #  namespace=namespace,
           # name="lifecycle_manager_navigation",
            #output="screen",
            #parameters=[
            #    {"use_sim_time": use_sim_time},
            #    {"autostart": True},
            #    {
            #        "node_names": [
            #            "controller_server",
            #            "behavior_server",
            #            "velocity_smoother",
             #       ]
             #   },
            #],
        #),
        

    ])
