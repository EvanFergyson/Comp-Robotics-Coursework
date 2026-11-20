"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Executes the launches to run amcl in the simulated demo.
"""

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Contains all the launch files and nodes to execute"""

    # ===========================================
    # ! PACKAGES DIR
    # ===========================================
    slam_examples_dir = FindPackageShare(package="slam_examples").find("slam_examples")

    # ===========================================
    # ! ARGUMENTS
    # ===========================================

    initial_pose_x_arg = DeclareLaunchArgument(
        name="initial_pose_x",
        default_value="0.0",
        description="Initial X position of the robot",
    )

    initial_pose_y_arg = DeclareLaunchArgument(
        name="initial_pose_y",
        default_value="0.0",
        description="Initial Y position of the robot",
    )

    initial_pose_yaw_arg = DeclareLaunchArgument(
        name="initial_pose_yaw",
        default_value="0.0",
        description="Initial Yaw orientation of the robot",
    )

    # ===========================================
    # ! NODES
    # ===========================================
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        output="screen",
        name="amcl",
        parameters=[
            slam_examples_dir + "/config/amcl_params.yaml",
            {
                "use_sim_time": True,
                "initial_pose.x": LaunchConfiguration("initial_pose_x"),
                "initial_pose.y": LaunchConfiguration("initial_pose_y"),
                "initial_pose.yaw": LaunchConfiguration("initial_pose_yaw"),
            },
        ],
    )

    # ===========================================
    # ! LAUNCH DESCRIPTION DECLARATION
    # ===========================================
    ld = LaunchDescription()

    # ===========================================
    # ! ADDING ARGUMENTS
    # ===========================================
    ld.add_action(initial_pose_x_arg)
    ld.add_action(initial_pose_y_arg)
    ld.add_action(initial_pose_yaw_arg)

    # ===========================================
    # ! ADDING NODES
    # ===========================================
    ld.add_action(amcl_node)

    return ld
