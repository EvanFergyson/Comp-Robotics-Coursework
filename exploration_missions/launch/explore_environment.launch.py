"""
Created on November 19, 2026

@author: TODO (your name and email)

Purpose: Launches the mission handler that runs the exploration mission
"""

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Contains all of the arguments and nodes to be launched"""

    # ===========================================
    # ! ARGUMENTS
    # ===========================================
    declare_n_waypoints = DeclareLaunchArgument(
        "n_waypoints",
        default_value="20",
        description="Number of random waypoints to visit.",
    )

    # ===========================================
    # ! NODES
    # ===========================================
    mission_handler_node = Node(
        package="exploration_missions",
        executable="mission_handler",
        name="mission_handler",
        output="screen",
        parameters=[{"n_waypoints": LaunchConfiguration("n_waypoints")}],
    )

    # ===========================================
    # ! LAUNCH DESCRIPTION DECLARATION
    # ===========================================
    ld = LaunchDescription()

    # ===========================================
    # ! ADDING ARGUMENTS
    # ===========================================
    ld.add_action(declare_n_waypoints)

    # ===========================================
    # ! ADDING NODES
    # ===========================================
    ld.add_action(mission_handler_node)

    return ld
