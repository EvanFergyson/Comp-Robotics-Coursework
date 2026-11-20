"""
Created on October 22, 2026

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Shows in RViz how a search-based planner explores the grid while it
solves a query
"""

import time
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

# expanded nodes shown per second, the planner is paused to keep to it so the
# search can be followed in RViz. A* and Dijkstra expand ~15000 nodes per
# second at full speed, raise it to watch the search faster
SEARCH_SPEED = 4000

# seconds between updates of the search in RViz
UPDATE_PERIOD = 0.1


class SearchVisualizer:
    """SearchVisualizer class

    Publishes the nodes expanded by a planner as cells of a marker on
    /planner_search. Each update adds a marker with the nodes expanded since
    the previous one, so the explored area grows in RViz while the planner
    searches"""

    def __init__(self, node):
        """SearchVisualizer constructor

        Declares the show_search parameter of the node, set from the launch
        file. When it is False the search is not shown and the planner runs
        at full speed.

        Args:
            node (rclpy.node.Node): node the marker publisher is created on
        """
        self.node = node
        self.show_search = node.declare_parameter("show_search", True).value
        self.marker_pub = node.create_publisher(Marker, "/planner_search", 10)
        # the planner blocks the node while it searches, so the updates are
        # published from expand(), this timer publishes the last expanded
        # nodes once the search is over
        node.create_timer(UPDATE_PERIOD, self.publish)

        self.cell_size = None
        self.shown = set()  # positions of the nodes already drawn
        self.new_cells = []  # expanded nodes waiting for the next update
        self.marker_id = 0
        self.last_update = 0.0

    def watch(self, planner):
        """Watch

        Clears the previous search from RViz and makes the planner report the
        nodes it expands. Called before each search of the planner, does
        nothing if show_search is False.

        Args:
            planner (AStar, Dijkstra or DStarLite): planner about to search,
            with its collision checker already set
        """
        if not self.show_search:
            return
        planner.search_visualizer = self
        self.cell_size = planner.collision_checker.map_resolution
        self.shown = set()
        self.new_cells = []
        self.marker_id = 0

        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = "map"
        self.marker_pub.publish(clear)

    def expand(self, x, y):
        """Expand

        Called by the planner for each node it expands

        Args:
            x (float): X position of the expanded node
            y (float): Y position of the expanded node
        """
        # a node expanded again is drawn once, overlapping cells look darker
        position = (round(x, 3), round(y, 3))
        if position in self.shown:
            return
        self.shown.add(position)
        self.new_cells.append(Point(x=x, y=y))

        if len(self.new_cells) >= SEARCH_SPEED * UPDATE_PERIOD:
            # pause the planner until the next update is due
            time.sleep(max(0.0, self.last_update + UPDATE_PERIOD - time.monotonic()))
        if time.monotonic() - self.last_update >= UPDATE_PERIOD:
            self.publish()

    def publish(self):
        """Publish

        Publishes the nodes expanded since the last update as a new marker,
        RViz keeps the previous ones so the whole search stays visible"""
        self.last_update = time.monotonic()
        if not self.new_cells:
            return

        marker = Marker()
        # stamp left at zero, RViz then uses the latest transform of map
        marker.header.frame_id = "map"
        marker.id = self.marker_id
        marker.type = Marker.CUBE_LIST
        # slightly above the map so the cells are drawn over it
        marker.pose.position.z = 0.01
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.cell_size
        marker.scale.y = self.cell_size
        marker.scale.z = 0.01
        marker.color.b = 1.0
        marker.color.a = 0.5
        marker.points = self.new_cells
        self.marker_pub.publish(marker)

        self.marker_id += 1
        self.new_cells = []
