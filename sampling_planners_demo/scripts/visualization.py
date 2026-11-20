"""
Created on November 9, 2026

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Shows in RViz how a sampling-based planner grows its tree while it
solves a query
"""

import time
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

# nodes added to the tree shown per second, the planner is paused to keep to
# it so the tree can be followed in RViz. RRT and RRT* add ~500-1500 nodes per
# second at full speed, raise it to watch the tree grow faster
SEARCH_SPEED = 500

# seconds between updates of the tree in RViz
UPDATE_PERIOD = 0.1


class SearchVisualizer:
    """SearchVisualizer class

    Publishes the tree of a planner as the lines of a marker on
    /planner_search. Each update redraws the whole tree, so RViz shows it
    growing, and the branches RRT* rewires, while the planner searches"""

    def __init__(self, node):
        """SearchVisualizer constructor

        Declares the show_search parameter of the node, set from the launch
        file. When it is False the tree is not shown and the planner runs at
        full speed.

        Args:
            node (rclpy.node.Node): node the marker publisher is created on
        """
        self.node = node
        self.show_search = node.declare_parameter("show_search", True).value
        self.marker_pub = node.create_publisher(Marker, "/planner_search", 10)
        # the planner blocks the node while it searches, so the updates are
        # published from grow(), this timer publishes the last update once the
        # search is over
        node.create_timer(UPDATE_PERIOD, self.publish)

        self.tree = None  # tree waiting to be drawn, None once it is drawn
        self.new_nodes = 0  # nodes added since the previous update
        self.last_update = 0.0

    def watch(self, planner):
        """Watch

        Clears the previous tree from RViz and makes the planner report its
        tree each time it grows. Called before each search of the planner,
        does nothing if show_search is False.

        Args:
            planner (RRT or RRTstar): planner about to search
        """
        if not self.show_search:
            return
        planner.search_visualizer = self
        self.tree = None
        self.new_nodes = 0

        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = "map"
        self.marker_pub.publish(clear)

    def grow(self, tree):
        """Grow

        Called by the planner each time a node is added to its tree

        Args:
            tree (tree.Tree): tree of the planner

        Returns:
            (float): seconds spent showing the tree, the planner adds them to
            its solve time so showing the tree does not change its result
        """
        start = time.monotonic()
        self.tree = tree
        self.new_nodes += 1

        if self.new_nodes >= SEARCH_SPEED * UPDATE_PERIOD:
            # pause the planner until the next update is due
            time.sleep(max(0.0, self.last_update + UPDATE_PERIOD - time.monotonic()))
        if time.monotonic() - self.last_update >= UPDATE_PERIOD:
            self.publish()
        return time.monotonic() - start

    def publish(self):
        """Publish

        Publishes the whole tree as a line from each node to its parent, the
        marker replaces the previous one"""
        self.last_update = time.monotonic()
        if self.tree is None:
            return

        marker = Marker()
        # stamp left at zero, RViz then uses the latest transform of map
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_LIST
        # above the map so the tree is drawn over it
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.color.b = 1.0
        marker.color.a = 1.0
        for node in self.tree.get_tree().values():
            if node.parent:
                marker.points.append(Point(x=node._x, y=node._y))
                marker.points.append(Point(x=node.parent._x, y=node.parent._y))
        self.marker_pub.publish(marker)

        self.tree = None
        self.new_nodes = 0
