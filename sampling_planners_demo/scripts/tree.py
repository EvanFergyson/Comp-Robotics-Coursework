"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Implementation of a Tree class to be used by sampling based algorithms
that grow trees
"""

import math
from node import Node


class Tree:
    """Tree class"""

    def __init__(self):
        """Tree constructor"""

        # self.nodes contains the tree in following manner:
        # {(x, y): Node, (x, y): Node, (x, y): Node}
        self.nodes = {}

    def set_start(self, _n: Node):
        """Set start

        Set the first node, the root of the tree

        Args:
            _n (Node): node to be set as the start for the tree
        """
        coords = (_n._x, _n._y)
        self.nodes[coords] = _n

    def get_tree(self):
        """Get tree

        Get the whole dictionary with the tree information

        Returns:
            {(x, y): Node}: dictionary with the tree information
        """
        return self.nodes

    def get_branch(self, _n):
        """Get branch

        Obtain nodes of branch in tree

        Args:
            _n (Node): node from which the branch will be obtained

        Returns:
            list(Node): list of nodes that form the branch from _n
        """
        branch = [_n]
        cur_node = _n
        while cur_node.parent:
            cur_node = self.nodes[(cur_node.parent._x, cur_node.parent._y)]
            branch.append(cur_node)
        branch.reverse()
        return branch

    def is_node_in_tree(self, _n):
        """Is node in tree

        Check if an specific node is in the tree

        Args:
            _n (Node): node to evaluate

        Returns:
            bool: whether the node has already been included or not in the tree
        """
        coords = (_n._x, _n._y)
        if coords in self.nodes.keys():
            return True
        return False

    def add_node(self, _n: Node):
        """Add node

        Add a new node to the tree

        Args:
            _n (Node): node to add to the tree
        """
        coords = (_n._x, _n._y)
        self.nodes[coords] = _n

    def neighbor_nodes(self, _n: Node, expansion_size):
        """Neighbor nodes

        Returns nodes inside the range of expansion size

        Args:
            _n (Node): node from which neighbors will be obtained
            expansion_size (float): expansion size used as radius to consider neighbors

        Returns:
            list(Node): list of neighbor nodes
        """
        nei_nodes = []
        for node in self.nodes:
            if _n.calculate_distance(self.nodes[node]) <= expansion_size + 0.000000001:
                nei_nodes.append(self.nodes[node])
        return nei_nodes

    def nearest(self, _n: Node):
        """Nearest

        Return the nearest node from _n

        Args:
            _n (Node): node to consider as reference to get nearest node

        Returns:
            Node: nearest node from _n
        """
        nearest_node = None
        dis = float("inf")

        for node in self.nodes:
            cur_dis = _n.calculate_distance(self.nodes[node])
            if cur_dis < dis:
                dis = cur_dis
                nearest_node = self.nodes[node]
        return nearest_node

    def step(self, nnear: Node, nrand: Node, expansion_size):
        """Step

        Defines the node step from nnear to nrand

        Args:
            nnear (Node): node from which the new node will be defined
            nrand (Node): random node
            expansion_size (float): distance consider to expand from nnear towards nrand

        Returns:
            Node: new node from the possible expansion
        """
        if nnear.calculate_distance(nrand) > expansion_size:
            theta = math.atan2(nrand._y - nnear._y, nrand._x - nnear._x)
            (_x, _y) = (
                nnear._x + expansion_size * math.cos(theta),
                nnear._y + expansion_size * math.sin(theta),
            )
            step_node = Node(_x, _y)
            return step_node
        return nrand

    def update_parent(self, node, _n):
        """Update parent

        Update the parent of an specific node in the tree

        Args:
            node (Node): node whose parent will be updated
            _n (Node): parent node
        """
        node_coord = (node._x, node._y)
        self.nodes[node_coord].parent = _n

    def cost(self, _n: Node):
        """Cost

        Returns the cost of a node to the start node

        Args:
            _n (Node): node from which the cost towards the start node
            will be calculated

        Returns:
            float: cost as distance in meters to move from _n
            towards the start of the tree
        """
        if not _n.parent:
            return 0
        cost = 0
        cur_node = _n
        next_node = self.nodes[(cur_node.parent._x, cur_node.parent._y)]
        cost += cur_node.calculate_distance(next_node)
        while next_node.parent:
            cur_node = next_node
            next_node = self.nodes[(cur_node.parent._x, cur_node.parent._y)]
            cost += cur_node.calculate_distance(next_node)
        return cost
