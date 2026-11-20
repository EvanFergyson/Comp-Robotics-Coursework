"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Executes the launches to run amcl in a real life demo.
"""

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


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
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"yaml_filename": slam_examples_dir + "/maps/map_real.yaml"},
        ],
    )

    lifecycle_nodes = ["map_server", "amcl"]
    use_sim_time = False
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

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        output="screen",
        name="amcl",
        parameters=[
            slam_examples_dir + "/config/amcl_real_params.yaml",
            {
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
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager_node)

    return ld
