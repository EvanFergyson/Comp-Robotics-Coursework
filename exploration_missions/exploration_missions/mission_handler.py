"""
Created on November 19, 2026

@author: TODO (your name and email)

Purpose: Mission handler for an exploration task. Generates random waypoints inside
the given bounds of the environment, sends them one at a time to the navigator and
saves the map once the mission is over.
"""

import math
import os
import random
import subprocess
import time
import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool


class MissionHandler(rclpy.node.Node):
    """MissionHandler class

    Only decides which waypoint to send next and when. The navigator plans the
    paths, the path follower moves the robot and nav2_map_server saves the map.
    """

    def __init__(self):
        """MissionHandler constructor"""

        super().__init__("mission_handler")

        # ===============================================================================
        # ! Parameters
        # ===============================================================================

        # dynamic typing so the bounds can be given as -5 or -5.0
        any_type = ParameterDescriptor(dynamic_typing=True)

        self.n_waypoints = self.declare_parameter("n_waypoints", 20).value

        # bounds of the environment in the map frame (origin = where the robot spawned)
        # TODO: replace with the house bounds measured with map_bounds.py
        x_min = float(self.declare_parameter("x_min", -4.2, any_type).value)
        x_max = float(self.declare_parameter("x_max", 9.9, any_type).value)
        y_min = float(self.declare_parameter("y_min", -6.0, any_type).value)
        y_max = float(self.declare_parameter("y_max", 3.9, any_type).value)

        # seconds before a waypoint the robot is not reaching is skipped
        self.waypoint_timeout = float(
            self.declare_parameter("waypoint_timeout", 180.0, any_type).value
        )

        # file name of the saved map, the date and time are appended to it
        self.map_path = self.declare_parameter("map_path", "~/exploration_map").value

        # ===============================================================================
        # ! Mission state
        # ===============================================================================

        # random waypoints inside the bounds of the environment
        self.waypoints = [
            (random.uniform(x_min, x_max), random.uniform(y_min, y_max))
            for _ in range(self.n_waypoints)
        ]

        self.robot_xy = (
            0.0,
            0.0,
        )  # last point the robot reached, starts at the map origin
        self.goal = None  # waypoint currently sent to the navigator
        self.path_received = (
            False  # True once the navigator has sent a path for self.goal
        )
        self.sent_time = 0.0  # when self.goal was sent
        self.reached = 0  # number of waypoints reached
        self.skipped = []  # waypoints that could not be reached
        self.done = False  # True once the map has been saved

        self.get_logger().info(
            f"Generated {self.n_waypoints} waypoints in "
            f"x [{x_min}, {x_max}], y [{y_min}, {y_max}]"
        )

        # =======================================
        #! PUBLISHERS
        # =======================================
        # sends each waypoint to the navigator, same message as RViz's 2D Goal Pose
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # =======================================
        #! SUBSCRIBERS
        # =======================================
        # the path follower publishes True when the robot has reached the goal
        self.create_subscription(Bool, "/goal_reached", self.goal_reached_callback, 10)

        # the navigator publishes a path with no poses when it cannot plan to the goal
        self.create_subscription(Path, "/path", self.path_callback, 10)

        # starts the mission and checks the waypoint timeout
        self.timer = self.create_timer(1.0, self.timer_callback)

    # =======================================
    #! CALLBACKS
    # =======================================
    def timer_callback(self):
        """Timer callback

        Sends the first waypoint once the navigator is running, then skips the
        current waypoint if the robot has not reached it within waypoint_timeout
        """
        if self.goal is None:
            # the navigator subscribes to /goal_pose, so a subscriber means it is running
            if self.goal_pub.get_subscription_count() > 0:
                self.send_next_waypoint()
        elif time.monotonic() - self.sent_time > self.waypoint_timeout:
            self.skip_waypoint("timed out")

    def path_callback(self, path: Path):
        """Path callback

        Listens to the paths sent by the navigator to the path follower

        Args:
            path (nav_msgs.msg.Path): solution path, with no poses if the navigator
            could not find a path to the goal
        """
        if self.goal is None or self.done:
            return

        if path.poses:
            self.path_received = True
        else:
            self.skip_waypoint("no path found")

    def goal_reached_callback(self, goal_reached: Bool):
        """Goal reached callback

        Sends the next waypoint once the path follower reports the goal as reached

        Args:
            goal_reached (std_msgs.msg.Bool): True when the robot has reached the goal
        """
        # a message that arrives before the path for this goal belongs to the previous one
        if goal_reached.data and self.path_received and not self.done:
            self.reached += 1
            self.robot_xy = self.goal
            self.send_next_waypoint()

    # =======================================
    #! MISSION
    # =======================================
    def send_next_waypoint(self):
        """Send next waypoint

        Sends the closest waypoint not yet visited to the navigator, or ends the
        mission if there are none left
        """
        if not self.waypoints:
            self.finish_mission()
            return

        # closest waypoint to the last point the robot reached
        self.goal = min(self.waypoints, key=lambda w: math.dist(w, self.robot_xy))
        self.waypoints.remove(self.goal)

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x, goal_msg.pose.position.y = self.goal
        goal_msg.pose.orientation.w = 1.0  # yaw of 0
        self.goal_pub.publish(goal_msg)

        self.path_received = False
        self.sent_time = time.monotonic()

        number = self.n_waypoints - len(self.waypoints)
        self.get_logger().info(
            f"Waypoint {number}/{self.n_waypoints} sent: "
            f"({self.goal[0]:.2f}, {self.goal[1]:.2f})"
        )

    def skip_waypoint(self, reason: str):
        """Skip waypoint

        Gives up on the current waypoint and sends the next one

        Args:
            reason (str): why the waypoint is skipped, shown in the log
        """
        self.get_logger().warning(
            f"Waypoint ({self.goal[0]:.2f}, {self.goal[1]:.2f}) skipped: {reason}"
        )
        self.skipped.append(self.goal)
        self.send_next_waypoint()

    def finish_mission(self):
        """Finish mission

        Saves the map with map_saver_cli (nav2_map_server) and shows a summary
        """
        self.done = True
        self.timer.cancel()

        map_name = os.path.expanduser(self.map_path) + time.strftime("_%Y%m%d_%H%M%S")
        result = subprocess.run(
            [
                "ros2",
                "run",
                "nav2_map_server",
                "map_saver_cli",
                "-f",
                map_name,
                "--ros-args",
                "-p",
                "save_map_timeout:=10.0",
            ]
        )

        skipped = [(round(x, 2), round(y, 2)) for x, y in self.skipped]
        self.get_logger().info(
            f"Mission complete: {self.reached}/{self.n_waypoints} waypoints reached, "
            f"{len(skipped)} skipped {skipped}"
        )
        if result.returncode == 0:
            self.get_logger().info(f"Map saved to {map_name}.pgm and {map_name}.yaml")
        else:
            self.get_logger().error("Map could not be saved")


def main(args=None):
    rclpy.init(args=args)
    mission_handler = MissionHandler()
    try:
        # spin until the mission is over
        while rclpy.ok() and not mission_handler.done:
            rclpy.spin_once(mission_handler)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        mission_handler.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
