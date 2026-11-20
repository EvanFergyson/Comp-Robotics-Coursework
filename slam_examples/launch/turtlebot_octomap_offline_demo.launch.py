"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Executes the demo with the turtlebot and octomap offline.
"""

import os
import tempfile
import yaml

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    """Contains all the nodes to execute"""

    # ===========================================
    # ! PACKAGES DIR
    # ===========================================
    slam_examples_dir = FindPackageShare(package="slam_examples").find("slam_examples")

    # ===========================================
    # ! ARGUMENTS
    # ===========================================
    rviz_ver_arg = DeclareLaunchArgument(
        name="rviz_ver",
        default_value="",
        description="RViz version config.",
    )

    map_x_arg = DeclareLaunchArgument(
        name="map_x",
        default_value="0.0",
        description="Offset in x added to the origin of the map to serve.",
    )

    map_y_arg = DeclareLaunchArgument(
        name="map_y",
        default_value="0.0",
        description="Offset in y added to the origin of the map to serve.",
    )

    map_yaw_arg = DeclareLaunchArgument(
        name="map_yaw",
        default_value="0.0",
        description=(
            "Rotation in yaw (rad) added to the origin of the map to serve. "
            "Only affects visualization, amcl and the costmaps ignore it."
        ),
    )

    # ===========================================
    # ! NODES
    # ===========================================

    nav_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            slam_examples_dir + "/rviz/nav_demo.rviz",
        ],
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration("rviz_ver"), '" == "nav"'])
        ),
    )

    slam_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            slam_examples_dir + "/rviz/slam_demo.rviz",
        ],
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration("rviz_ver"), '" == "slam"'])
        ),
    )

    def create_map_server_node(context):
        """Serves the map from a copy of its yaml with a displaced origin."""

        map_yaml = slam_examples_dir + "/maps/mymap.yaml"

        map_offset = [
            float(LaunchConfiguration(name).perform(context))
            for name in ("map_x", "map_y", "map_yaw")
        ]

        if any(map_offset):
            with open(map_yaml, "r") as original_file:
                map_data = yaml.safe_load(original_file)

            # The image is relative to the original yaml, so it has to be
            # absolute once the copy lives somewhere else.
            map_data["image"] = os.path.join(
                os.path.dirname(map_yaml), map_data["image"]
            )
            map_data["origin"] = [
                value + offset
                for value, offset in zip(map_data["origin"], map_offset)
            ]

            map_yaml = os.path.join(tempfile.gettempdir(), "mymap_misplaced.yaml")
            with open(map_yaml, "w") as misplaced_file:
                yaml.safe_dump(map_data, misplaced_file)

        return [
            Node(
                package="nav2_map_server",
                executable="map_server",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"yaml_filename": map_yaml},
                ],
            )
        ]

    map_server_node = OpaqueFunction(function=create_map_server_node)

    lifecycle_nodes = ["map_server", "amcl"]
    use_sim_time = True
    autostart = True

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    # ===========================================
    # ! LAUNCHES
    # ===========================================

    tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [slam_examples_dir, "/launch/", "turtlebot3_house.launch.py"]
        ),
        launch_arguments={"x_pose": "-3.0", "y_pose": "1.0"}.items(),
    )

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [slam_examples_dir, "/launch/", "amcl.launch.py"]
        ),
        launch_arguments={"initial_pose_x": "-3.0", "initial_pose_y": "1.0"}.items(),
    )

    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [slam_examples_dir, "/launch/", "octomap_offline.launch.py"]
        ),
    )

    # ===========================================
    # ! LAUNCH DESCRIPTION DECLARATION
    # ===========================================
    ld = LaunchDescription()

    ld.add_action(rviz_ver_arg)
    ld.add_action(map_x_arg)
    ld.add_action(map_y_arg)
    ld.add_action(map_yaw_arg)

    # ===========================================
    # ! ADDING NODES
    # ===========================================
    ld.add_action(nav_rviz_node)
    ld.add_action(slam_rviz_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)

    # ===========================================
    # ! ADDING LAUNCHES
    # ===========================================
    ld.add_action(tb3_launch)
    ld.add_action(amcl_launch)
    ld.add_action(octomap_launch)

    return ld
