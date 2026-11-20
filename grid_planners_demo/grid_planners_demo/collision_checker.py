"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: class to perform collision checking using a grid.
"""

import math
import rclpy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from grid_planners_demo.node import Node


class CollisionChecker:
    """CollisionChecker class

    Collision checker to check if nodes are free of collision or not. Uses OccupancyGrid in order to
    check the collision based on robot radius. Contains additional function to get indexes from the
    OccupancyGrid and coordinates.
    """

    def __init__(
        self,
        grid_map: OccupancyGrid,
        robot_radius: float,
        planning_bounds: list = [20, 20],
        opportunistic_cc: bool = True,
    ):
        """CollisionChecker constructor"""

        # map data used for collision checking
        self.map_data = grid_map.data
        self.map_width = grid_map.info.width
        self.map_height = grid_map.info.height
        self.map_resolution = grid_map.info.resolution
        self.map_origin = grid_map.info.origin.position

        # other variables
        self.robot_radius = robot_radius
        self.planning_bounds = planning_bounds
        self.tolerance = math.ceil(self.robot_radius / self.map_resolution)

        # if opportunistic collision checking is used, then the grid_map is
        # expanded with unoccupied values in the new areas
        if opportunistic_cc:
            self.expand_grid_map()

    def expand_grid_map(self):
        """Expand grid map

        Expands the grid map with unnoccupied values, useful to perform opportunistic
        collision checking
        """
        if len(self.planning_bounds) > 1:
            # new map
            map_width = int(self.planning_bounds[0] / self.map_resolution)
            map_height = int(self.planning_bounds[1] / self.map_resolution)
            res = self.map_resolution
            di = round((self.map_origin.x + map_width * res / 2) / res)
            dj = round((self.map_origin.y + map_height * res / 2) / res)
            map_origin = Point()
            map_origin.x = self.map_origin.x - di * res
            map_origin.y = self.map_origin.y - dj * res

            # a new map with same dimensions is created
            map_data = [0] * map_width * map_height

            # the new map is populated with the values from the previous map
            for i in range(0, self.map_width):
                for j in range(0, self.map_height):
                    value = self.get_by_indices(i, j)
                    if value == 100:
                        ni_, nj_ = i + di, j + dj
                        # obstacles outside the planning bounds are dropped
                        if 0 <= ni_ < map_width and 0 <= nj_ < map_height:
                            map_data[nj_ * map_width + ni_] = 100

            # the new map is added to the map variables of the class
            self.map_data = map_data
            self.map_width = map_width
            self.map_height = map_height
            self.map_origin = map_origin

    def get_by_indices(self, i: int, j: int) -> int:
        """Obtain index of OccupancyGrid data according to i and j from grid.

        Args:
            i (int): index in x
            j (int): index in j

        Returns:
            int: index from data list in OccupancyGrid
        """
        index = None
        try:
            index = self.map_data[j * self.map_width + i]
        except IndexError:
            rclpy.logging.get_logger("Collision Checker").warning(
                "Position not available in map"
            )
        return index

    def get_by_coordinates(self, x: float, y: float) -> int:
        """Obtain index of OccupancyGrid data according to i and j from grid.

        Args:
            x (float): x coordinate in the world
            y (float): y coordinate in the world

        Returns:
            int: index from data list in OccupancyGrid
        """
        indices = self.coordinates_to_indices(x, y)
        return self.get_by_indices(indices[0], indices[1])

    def coordinates_to_indices(self, x: float, y: float) -> tuple:
        """Transform coordinates to indices of the grid map

        Args:
            x (float): x coordinate of the grid map
            y (float): y coordinate of the grid map

        Returns:
            i, j (tuple): tuple of the indexes
        """
        i = math.floor((x - self.map_origin.x) / self.map_resolution)
        j = math.floor((y - self.map_origin.y) / self.map_resolution)
        return i, j

    def indices_to_coordinates(self, i: int, j: int) -> tuple:
        """Transforms indices to coordinates of the grid map

        Args:
            i (int): index in x axis grid map
            j (int): index in y axis grid map

        Returns:
            x, y (tuple): tuple of the coordinates
        """
        x = i * self.map_resolution + self.map_origin.x
        y = j * self.map_resolution + self.map_origin.y
        return x, y

    def is_node_free(self, node: Node) -> bool:
        """Check if a node is collision free according to its position in the grid_map

        Args:
            node (Node): the node to check if collision free

        Returns:
            bool: Whether the node is collision free or not
        """
        i, j = self.coordinates_to_indices(node._x, node._y)

        # the whole footprint must be inside the map, or indices wrap to other rows
        tol = self.tolerance
        if not tol <= i < self.map_width - tol or not tol <= j < self.map_height - tol:
            return False

        # square footprint, tol cells on every side of the node's cell
        for offset_x in range(-tol, tol + 1):
            for offset_y in range(-tol, tol + 1):
                val = self.get_by_indices(i - offset_x, j - offset_y)
                if val is None:
                    return False
                if not -1 < val < 100:
                    return False
        return True

    def is_path_free(self, path):
        """Checks whether a path is free of collision or not by
        returning the list of nodes that are in collision.

        Args:
            path (list(Node)): list of Node objects

        Returns:
            occ_nodes (list((x, y))): list of coordinates as tuples with the nodes the
            positions that are now occupied or in collision
        """
        occ_nodes = []

        for node in path:
            if not self.is_node_free(node):
                occ_nodes.append(self.coordinates_to_indices(node._x, node._y))
        if len(occ_nodes) > 0:
            rclpy.logging.get_logger("Collision Checker").warning(
                "Current path is not collision free, calculating a new path"
            )

        return occ_nodes
