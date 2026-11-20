"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Executes the node of gmapping for the robot in simulation.
"""

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():
    """Contains all the nodes to execute"""

    # ===========================================
    # ! PACKAGES DIR
    # ===========================================
    slam_examples_dir = FindPackageShare(package="slam_examples").find("slam_examples")

    # ===========================================
    # ! NODES
    # ===========================================
    gmapping_node = Node(
        package="slam_gmapping",
        executable="slam_gmapping",
        output="screen",
        name="turtlebot3_slam_gmapping",
        parameters=[slam_examples_dir + "/config/gmapping_params.yaml"],
    )

    # ===========================================
    # ! LAUNCH DESCRIPTION DECLARATION
    # ===========================================
    ld = LaunchDescription()

    # ===========================================
    # ! ADDING NODES
    # ===========================================
    ld.add_action(gmapping_node)

    return ld
