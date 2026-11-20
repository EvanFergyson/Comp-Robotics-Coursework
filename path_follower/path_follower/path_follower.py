"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Path following node that moves the robot through a given set of positions.
"""

import math
import time
import rclpy
import rclpy.node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from numpy import nanmin
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformListener, Buffer
from path_follower.node import Node

MOVE_TOLERANCE = 0.1
SCAN_TOLERANCE_FRONT = 0.4
SCAN_TOLERANCE_SIDE = 0.3
ROTATE_TOLERANCE = 0.004


class PathFollower:
    """Path follower class
    Follows the path obtained from a specific topic until the final position."""

    def __init__(self):
        """Path follower constructor"""

        # class does not inherit from rclpy.node.Node
        # in this case we handle the node from the class constructor
        # so we have more control onto when to spin the node
        self.node = rclpy.node.Node("path_follower_node")

        # ===============================================================================
        # ! Initial values
        # ===============================================================================

        # parameters
        self.goal_tolerance = self.node.declare_parameter("goal_tolerance", 0.1).value
        self.linear_velocity = self.node.declare_parameter(
            "linear_velocity", 0.25
        ).value
        self.angular_velocity = self.node.declare_parameter(
            "angular_velocity", 1.5
        ).value

        self.robot_position = None  # stores current position of the robot
        # Used to avoid obstacles, currently unused for the navigation examples
        self.path_deviation = 0.0

        self.goal = PoseStamped()  # stores final goal for the robot

        self.following_path = []  # stores path to be followed

        #! Flags to handle the robots state and path following
        self.is_shutdown_initiated = False
        self.is_moving = False
        self.is_obstacle_ahead = False
        self.new_path_arrived = False

        # TF listener initialization
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self.node)

        # =======================================
        #! SUBSCRIBERS
        # =======================================

        # A Path message to get the positions to be followed
        self.path_sub = self.node.create_subscription(
            Path, "/path", self.path_callback, 10
        )

        # A LaserScan message to get the distances from surrounding obstacles
        # as measured by the simulater lidar
        self.scan_sub = self.node.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        # A Bool message that defined if the motion should be stopped
        self.motion_stop_sub = self.node.create_subscription(
            Bool, "/stop_motion", self.stop_motion_callback, 10
        )

        # A TFMessage mesage that defines when a TF is being updated
        # this is only done to continuously update the position of the robot
        self.tf_sub = self.node.create_subscription(
            TFMessage, "/tf", self.robot_pose_callback, 10
        )

        # A PoseStamped message that defines the goal for the controller
        self.goal_sub = self.node.create_subscription(
            PoseStamped, "/goal_controller", self.goal_callback, 10
        )

        # =======================================
        #! PUBLISHERS
        # =======================================
        # A TwistStamped message to send the velocity for the robot to move
        self.velocity_publisher = self.node.create_publisher(
            TwistStamped, "/cmd_vel", 10
        )

        # A Bool message to notify if the robot is moving
        self.motion_publisher = self.node.create_publisher(Bool, "/robot_is_moving", 10)

        # A Bool message to notify when the robot has reached the goal
        self.goal_reached_publisher = self.node.create_publisher(
            Bool, "/goal_reached", 10
        )

    def publish_velocity(self, vel_msg: TwistStamped = None):
        """Publish velocity

        Stamps the velocity command with the current time and publishes it.
        Called without arguments it publishes a zero velocity, stopping the robot.

        Args:
            vel_msg (geometry_msgs.msg.TwistStamped, optional): velocity command to send.
            Defaults to a zero velocity.
        """
        if vel_msg is None:
            vel_msg = TwistStamped()

        vel_msg.header.stamp = self.node.get_clock().now().to_msg()
        vel_msg.header.frame_id = "base_link"

        self.velocity_publisher.publish(vel_msg)

    def goal_callback(self, pose_msg: PoseStamped):
        """Goal callback

        Obtain goal and store it to know when the
        destination has been reached

        Args:
            pose_msg (geometry_msgs.msg.PoseStamped): message with the information of the
            goal"""
        self.goal = Node.from_pose(pose_msg.pose)

    def robot_pose_callback(self, data: TFMessage):
        """Robot pose callback

        Listens to robot current pose and stores it"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            position = transform.transform.translation
            quaternion = transform.transform.rotation
        except:
            return

        position = [
            position.x,
            position.y,
            position.z,
        ]
        orientation = [
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ]

        self.robot_position = Node.from_tf(position, orientation)

    def stop_motion_callback(self, stop_motion: Bool):
        """Stop motion callback

        Listens to a flag if the motion has to be stopped"""
        if stop_motion.data:
            self.stop_moving()
            self.initialize_stop()

    def path_callback(self, path: Path):
        """Path callback

        Listens to obtained solution path from path planner

        Args:
            path (nav_msgs.msg.Path): contains the poses that are given by the path planner
            and published by the navigator
        """

        # the nearest node is computed against the robot pose, so the path cannot
        # be processed until the map -> base_link transform has been received
        if self.robot_position is None:
            self.node.get_logger().warn(
                "Robot pose is not available yet, discarding path"
            )
            return

        self.new_path_arrived = True
        node_path = []  # variable where nodes will be initially stored

        nearest_node = 0  # get nearest node from robot to follow
        distance = float("inf")

        # add all the nodes from the nearest position from the robot
        for i in range(0, len(path.poses)):
            node = Node.from_pose(path.poses[i].pose)
            node_path.append(node)
            node_distance = node.calculate_distance(self.robot_position)
            if node_distance < distance:
                distance = node_distance
                nearest_node = i

        node_path = node_path[nearest_node:]
        self.following_path = node_path
        self.is_shutdown_initiated = False

    def scan_callback(self, scan_data: LaserScan):
        """Scan callback

        Listens to the scan obtained from the lidar

        Args:
            scan_data (sensor_msgs.msg.LaserScan): contains the values measured by the lidar
        """
        if (
            nanmin(scan_data.ranges[0:10] + scan_data.ranges[350:360])
            < SCAN_TOLERANCE_FRONT
        ):
            self.is_obstacle_ahead = False
        elif nanmin(scan_data.ranges[11:165]) < SCAN_TOLERANCE_SIDE:
            self.path_deviation = 0
        elif nanmin(scan_data.ranges[195:349]) < SCAN_TOLERANCE_SIDE:
            self.path_deviation = 0
        else:
            self.path_deviation = 0

    def initialize_stop(self):
        """Initialize stop

        Defines a flag so that the path following is shutdown"""
        self.is_shutdown_initiated = True

    def stop_moving(self):
        """Stop moving

        Stops the movement of the robot by publishing an empty TwistStamped message"""
        self.following_path = []
        self.is_shutdown_initiated = False
        self.is_moving = False
        self.publish_velocity()

    def follow_path(self):
        """Follow path

        Follows the path previously stored"""
        self.is_moving = True

        # while there are poses to follow, the next node in
        # self.following_path is taken
        while len(self.following_path) > 0:
            if self.is_shutdown_initiated:
                self.stop_moving()
                return
            node_to_follow = self.following_path.pop(0)
            self.move_to_point(node_to_follow)

        # check if the robot has already arrived to the goal
        # with a certain tolerance
        if (
            math.sqrt(
                math.pow(self.goal._x - self.robot_position._x, 2)
                + math.pow(self.goal._y - self.robot_position._y, 2)
            )
            < self.goal_tolerance + 0.1
        ):
            self.publish_velocity()  # Stopping robot
            self.rotate_to_goal(self.goal)  # rotate according to goal orientation
            self.publish_velocity()  # stop the robot

            self.goal_reached_publisher.publish(
                Bool(data=True)
            )  # notify the robot has reached the goal
            self.node.get_logger().info("Goal has been reached")
            self.is_moving = False

    def move_to_point(self, point: Node):
        """Move to point

        Moves the robot towards the point given by sending angular and linear velocities

        Args:
            point (Node): contains the pose to be followed.
        """
        vel_msg = TwistStamped()
        # the velocities live under .twist in a TwistStamped message
        vel_msg.twist.linear.x = self.linear_velocity  # linear velocity is a constant

        # while there is a certain distance to the point of interest
        while self.robot_position.calculate_distance(point) > MOVE_TOLERANCE:
            if (
                self.is_shutdown_initiated
                or self.is_obstacle_ahead
                or self.new_path_arrived
            ):
                self.new_path_arrived = False
                return

            # define angular velocity according to an orientation error
            speed = self.angular_velocity * self.angular_difference(point)
            vel_msg.twist.angular.z = (
                min(self.angular_velocity, speed) + self.path_deviation
            )

            self.publish_velocity(vel_msg)
            time.sleep(0.001)
            rclpy.spin_once(self.node)  # spin the node for all callbacks to update

    def rotate_to_goal(self, goal: Node):
        """Rotate to goal

        Args:
            goal (Node): contains information of the robot's goal
        """
        # if the incoming variable is of another type, it is changed to Node
        if isinstance(goal, PoseStamped):
            goal = Node.from_pose(goal.pose)

        # define velocity message
        vel_msg = TwistStamped()

        # while there is an error in the orientation
        while abs(goal.theta - self.robot_position.theta) > ROTATE_TOLERANCE:
            if self.is_shutdown_initiated:
                self.stop_moving()
                return

            # define angular speed to rotate the robot
            speed = self.angular_velocity * (goal.theta - self.robot_position.theta)
            vel_msg.twist.angular.z = min(self.angular_velocity, speed)
            self.publish_velocity(vel_msg)
            time.sleep(0.001)
            rclpy.spin_once(self.node)  # spin the node for all callbacks to update

        self.publish_velocity()  # stop the robot once the orientation is correct

    def angular_difference(self, point: Node) -> float:
        """Angular difference

        Args:
            point (Node): contains information of the node to be used to calculate the angular difference

        Returns:
            float: _description_
        """
        angle = (
            math.atan2(
                point._y - self.robot_position._y, point._x - self.robot_position._x
            )
            - self.robot_position.theta
        )

        if angle <= -math.pi:  # Normalizing angle
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi

        return angle


def main(args=None):
    rclpy.init(args=args)

    controller = PathFollower()

    try:
        while rclpy.ok():
            rclpy.spin_once(controller.node)
            if len(controller.following_path) > 0:
                controller.follow_path()

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass

    finally:
        controller.node.destroy_node()
        # on Ctrl-C the signal handler already shut down the context, and
        # calling shutdown() a second time raises RCLError
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
