"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Launches all of the needed nodes to make the robot navigate using the RRT* algorithm
"""

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """Contains all of the parameters and nodes to be launched"""

    # ===========================================
    # ! PACKAGES DIRECTORIES
    # ===========================================
    sampling_planners_demo_dir = FindPackageShare(
        package="sampling_planners_demo"
    ).find("sampling_planners_demo")

    # ===========================================
    # ! ARGUMENTS
    # ===========================================
    declare_show_search = DeclareLaunchArgument(
        "show_search",
        default_value="false",
        description="Show in RViz how the planner grows its tree.",
    )

    # Added by Evan Fergyson
    declare_online = DeclareLaunchArgument(
        "online",
        default_value="false",
        description="Replan as new obstacles are mapped.",
    )

    # ===========================================
    # ! NODES
    # ===========================================
    navigator_node = Node(
        package="sampling_planners_demo",
        executable="navigator_rrt_star.py",
        name="navigator_node",
        output="screen",
        parameters=[
            sampling_planners_demo_dir + "/config/rrt_star_params.yaml",
            {
                "show_search": LaunchConfiguration("show_search"),
                "online": LaunchConfiguration("online"),
            },
        ],
    )

    path_follower_node = Node(
        package="path_follower",
        executable="path_follower",
        name="path_follower_node",
        output="screen",
        parameters=[{"linear_velocity": 0.3, "angular_velocity": 2.5}],
        condition=IfCondition(LaunchConfiguration("enable_move", default=True)),
    )

    # ===========================================
    # ! LAUNCH DESCRIPTION DECLARATION
    # ===========================================
    ld = LaunchDescription()

    # ===========================================
    # ! ADDING ARGUMENTS
    # ===========================================
    ld.add_action(declare_show_search)
    ld.add_action(declare_online)

    # ===========================================
    # ! ADDING NODES
    # ===========================================
    ld.add_action(navigator_node)
    ld.add_action(path_follower_node)

    return ld
