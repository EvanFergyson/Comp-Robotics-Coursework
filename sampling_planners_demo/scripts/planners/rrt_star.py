"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: RRT* algorithm implementation used for path planning
"""

import math
import time
from functools import wraps
import random
import rclpy
from node import Node
from tree import Tree
from collision_checker import CollisionChecker


def timeit(func):
    """timeit

    This method is used to obtain the time used by a specific method. Has to be used
    as a method decorator. Typically used in this example to obtain the time taken
    by the solve method of the algorithm implementation"""

    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        rclpy.logging.get_logger("RRTstar planner").info(
            f"Function {func.__name__}{args} {kwargs} Took {total_time:.4f} seconds"
        )
        return result

    return timeit_wrapper


class RRTstar:
    """RRTstar class

    RRTstar algorithm implementation as a planner to solve start-to-goal queries"""

    def __init__(self):
        """RRTstar constructor"""

        # initial states of the robot
        self.start: Node = None
        self.goal: Node = None

        # algorithm variables
        self.step_size = 0.001
        self.expansion_size = 0.5
        self.bias = 0.5

        # collision checker and other variables
        self.collision_checker: CollisionChecker = None
        # tree class used by the algorithm
        # stores a parent-child node structure
        self.tree = Tree()

        self.tolerance = None  # tolerance considered for solution path found
        self.solution_node = None  # node considered that has reached the goal
        self.solution_path = None  # list of nodes considered as the solution path
        # set by SearchVisualizer.watch() to show the tree growing in RViz
        self.search_visualizer = None

    # =============================
    # ! SETTERS
    # =============================
    def set_start(self, start):
        """Set start

        Set start position of the robot

        Args:
            start (node.Node): start node for the planning problem.
        """

        self.start = start

    def set_goal(self, goal):
        """Set goal

        Set goal position of the robot

        Args:
            goal (node.Node): goal node for the planning problem.
        """
        self.goal = goal

    def set_collision_checker(self, collision_checker):
        """Set collision checker

        Set collision checker to detect collision free nodes

        Args:
            collision_checker (collision_checker.CollisionChecker): the object used to perform
            collision checking by the algorithm
        """
        self.collision_checker = collision_checker

    def set_goal_tolerance(self, tolerance):
        """Set goal tolerance

        Set goal tolerance to check when the query has been solved

        Args:
            tolerance (float): distance from the goal in which the query can be considered
            as solved
        """
        self.tolerance = tolerance

    def set_step_size(self, step_size):
        """Set step size

        Set the step size to determine if connection between two
        nodes is possible with collision checking

        Args:
            step_size (float): step size in meters to check connection between to nodes.
        """
        self.step_size = step_size

    def set_expansion_size(self, expansion_size):
        """Set expansion size

        Set the maximum expansion for a new node in the tree

        Args:
            expansion_size (float): expansion size considered to generate a new node
            considering the nearest node from the tree
        """

        self.expansion_size = expansion_size

    def set_bias(self, bias):
        """Set bias

        Set the bias of the planner

        Args:
            bias (float): should be a value between 0 and 1, defines the probability
            of biasing towards the goal when expanding the tree
        """
        self.bias = bias

    # =============================
    # ! GETTERS
    # =============================
    def get_solution_path(self):
        """Get solution path

        Obtain stored solution path

        Returns:
            list(node.Nodes): list of nodes as part of the solution path"""
        return self.solution_path

    def get_path_length(self):
        """Get path length

        Obtain the length of the solution path

        Returns:
            float: length calculated from the solution path
        """
        distance = 0

        for i in range(0, len(self.solution_path) - 2):
            distance += math.sqrt(
                math.pow(self.solution_path[i]._x - self.solution_path[i + 1]._x, 2)
                + math.pow(self.solution_path[i]._y - self.solution_path[i + 1]._y, 2)
            )

        return distance

    def get_tree(self):
        """Get tree

        Obtain the tree structure as a dictionary

        Returns:
            dict((float, float): node.Node): the tree built from using the Tree class
        """
        tree = self.tree.get_tree()
        return tree

    # =============================

    def sample(self):
        """Sample

        Samples a random node withing the configuration space defined.
        In this case between the boundaries of the map considered
        for the navigation.

        Returns:
            node.Node: returns a node with random X and Y values which is the current
            configuration space considered for the demos. An R2 configuration space.
        """

        _i = int(random.uniform(0, self.collision_checker.map_width - 1))
        _j = int(random.uniform(0, self.collision_checker.map_height - 1))
        _x, _y = self.collision_checker.indices_to_coordinates(_i, _j)
        node = Node(_x, _y)
        return node

    def check_and_reconnect(self, _n: Node, neighbors):
        """Check and reconnect

        Checks and reconnects nearby neighbors with _n depending on cost

        Args:
            _n (node.Node): node that will be taken as reference for the reconnection
            neighbors (list(node.Node)): neighbor nodes from _n to be considered for
            reconnection
        """
        for node in neighbors:
            cur_cost = self.tree.cost(node)
            new_cost = self.tree.cost(_n) + _n.calculate_distance(node)
            if new_cost < cur_cost and self.collision_checker.is_connection_free(
                node, _n, self.step_size
            ):
                self.tree.update_parent(node, _n)

    @timeit
    def solve(self, termination_time) -> list:
        """Solve

        Function to find path from start to goal using RRT* algorithm

        Args:
            termination_time (float): time in seconds given to the planner
            to find a solution path
        """
        # ! CHECK IF ALL THE REQUIRED ATTRIBUTES HAVE BEEN SET
        if not self.collision_checker:
            rclpy.logging.get_logger("RRTstar planner").warn(
                "Collision checker has not been set on the planner"
            )
            return False
        if not self.start:
            rclpy.logging.get_logger("RRTstar planner").warn(
                "Start position has not been set on the planner"
            )
            return False
        if not self.goal:
            rclpy.logging.get_logger("RRTstar planner").warn(
                "Goal position has not been set on the planner"
            )
            return False

        # ! START SOLVING THE QUERY
        timeout = time.time() + termination_time  # define the timeout time

        # initialise first nodes
        self.tree.add_node(self.start)  # add start node into the tree

        while timeout > time.time():  # while the timeout is not accomplished
            # generate random number between 0 and 1 to decide bias
            temp_val = random.uniform(0, 1)
            if 0 < temp_val < self.bias:
                nrand = self.goal  # bias against goal
            else:
                nrand = self.sample()  # random node

            nnear = self.tree.nearest(nrand)  # select nearest node in the tree
            new_node = self.tree.step(
                nnear, nrand, self.expansion_size
            )  # define the new node from the nearest node to random node

            # check collision of new node
            if self.collision_checker.is_node_free(new_node):
                # obtain neighbors from new node
                neighbors = self.tree.neighbor_nodes(new_node, self.expansion_size)
                cur_cost = float("inf")
                parent_node = None

                # connect to the neighbor (parent) with the lowest cost
                for node in neighbors:
                    new_cost = self.tree.cost(node) + new_node.calculate_distance(node)

                    if new_cost < cur_cost:
                        cur_cost = new_cost
                        parent_node = node
                # once the parent node is decided, check that the connection is possible
                if self.collision_checker.is_connection_free(
                    new_node, parent_node, self.step_size
                ) and not self.tree.is_node_in_tree(new_node):
                    # if connection is possible, add it to the tree structure
                    new_node.parent = parent_node
                    self.tree.add_node(new_node)
                    # then reconnect the nodes in the neighbor of the new node
                    self.check_and_reconnect(new_node, neighbors)
                    # show the tree growing in RViz, the time it takes is added
                    # to the timeout so the planner keeps its solve time
                    if self.search_visualizer:
                        timeout += self.search_visualizer.grow(self.tree)

                    # check if the new node is already a tolerance distance from the goal
                    # and no previous solution has been found
                    if (
                        new_node.calculate_distance(self.goal) < self.tolerance
                        and self.solution_node is None
                    ):
                        # if it is a tolerance distance from the goal, and initial solution
                        # has been found, the algorithm will keep going to improve
                        # the current solution

                        # print information about the initial solution
                        self.solution_node = new_node
                        rclpy.logging.get_logger("RRTstar planner").info(
                            "Initial solution found at: "
                            + str(time.time() - (timeout - termination_time))
                            + " seconds."
                        )
                        self.solution_path = self.tree.get_branch(self.solution_node)
                        init_path_length = self.tree.cost(self.solution_node)
                        rclpy.logging.get_logger("RRTstar planner").info(
                            "Initial solution path length: "
                            + str(init_path_length)
                            + " meters."
                        )

        # if solution has been found return True
        # otherwise False
        if self.solution_node:
            self.solution_path = self.tree.get_branch(self.solution_node)
            return True
        return False
