"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Executes octomap to run offline.
"""

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Contains all the nodes to execute"""

    # ===========================================
    # ! PACKAGES DIR
    # ===========================================
    slam_examples_dir = FindPackageShare(package="slam_examples").find("slam_examples")

    # ===========================================
    # ! ARGUMENTS
    # ===========================================
    octomap_res_arg = DeclareLaunchArgument(
        name="octomap_res",
        default_value="0.05",
        description="Resolution of octomap to load.",
    )

    # ===========================================
    # ! NODES
    # ===========================================
    octomap_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[
            slam_examples_dir + "/config/octomap_params.yaml",
            {
                "frame_id": "map",
                "use_height_map": True,
                "octomap_path": [
                    slam_examples_dir,
                    "/maps/map",
                    LaunchConfiguration("octomap_res"),
                    ".bt",
                ],
            },
        ],
        remappings=[("/cloud_in", "/depth_camera/depth/points")],
    )

    # ===========================================
    # ! LAUNCH DESCRIPTION DECLARATION
    # ===========================================
    ld = LaunchDescription()

    # ===========================================
    # ! ADDING ARGUMENTS
    # ===========================================
    ld.add_action(octomap_res_arg)

    # ===========================================
    # ! ADDING NODES
    # ===========================================
    ld.add_action(octomap_node)

    return ld
