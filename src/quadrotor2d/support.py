from shapely.geometry import Point, Polygon, LineString, box
from environment import Environment, plot_environment, plot_line, plot_poly
import numpy as np

# from assignment 3, given in search_classes.py
class SearchNode(object):
    def __init__(self, state, parent_node=None, cost=0.0, action=None, orientation=0.0, velocity=0.0):
        self._parent = parent_node
        self._state = state
        self._action = action
        self._cost = cost

        self._orientation = orientation ## TODO ME
        self._velocity    = velocity ## TODO ME


    def __repr__(self):
        return "<SearchNode (id: %s)| state: %s, cost: %s, parent_id: %s>" % (id(self), self.state, self.cost, id(self.parent))

    @property
    def state(self):
        """Get the state represented by this SearchNode"""
        return self._state

    @property
    def parent(self):
        """Get the parent search node that we are coming from."""
        return self._parent

    @property
    def cost(self):
        """Get the cost to this search state"""
        return self._cost

    @property
    def action(self):
        """Get the action that was taken to get from parent to the state represented by this node."""
        return self._action

    ## TODO ME
    @property
    def orientation(self):
        """Get the action that was taken to get from parent to the state represented by this node."""
        return self._orientation

    ## TODO ME
    @property
    def velocity(self):
        """Get the action that was taken to get from parent to the state represented by this node."""
        return self._velocity

    def __eq__(self, other):
        return isinstance(other, SearchNode) and self._state == other._state

    def __hash__(self):
        return hash(self._state)

    def __gt__(self, other):
        return self._cost > other._cost

# from assignment 3, given in search_classes.py
class Path(object):
    """This class computes the path from the starting state until the state specified by the search_node
    parameter by iterating backwards."""

    def __init__(self, search_node):
        self.path = []
        node = search_node
        while node is not None:
            self.path.append(node.state)
            node = node.parent
        self.path.reverse()
        self.cost = search_node.cost

    def __repr__(self):
        return "Path of length %d, cost: %.3f: %s" % (len(self.path), self.cost, self.path)

    def edges(self):
        return zip(self.path[0:-1], self.path[1:])

# from assignment 3, given in graph.py
# import pydot_ng as pydot
# import networkx as nx
import matplotlib.pyplot as plt

class NodeNotInGraph(Exception):
    def __init__(self, node):
        self.node = node

    def __str__(self):
        return "Node %s not in graph." % str(self.node)

# from assignment 3, given
class Edge(object):
    def __init__(self, source, target, weight=1.0):
        self.source = source
        self.target = target
        self.weight = weight

    def __hash__(self):
        return hash("%s_%s_%f" % (self.source, self.target, self.weight))

    def __eq__(self, other):
        return self.source == other.source and self.target == other.target \
               and self.weight == other.weight

    def __repr__(self):
        return "Edge(%r,%r,%r)" % (self.source, self.target, self.weight)

# from assignment 3, given
class Graph(object):
    def __init__(self, node_label_fn=None):
        self._nodes = set()
        self._edges = dict()
        self.node_label_fn = node_label_fn if node_label_fn else lambda x: x
        self.node_positions = dict()

    def __contains__(self, node):
        return node in self._nodes

    def add_node(self, node):
        """Adds a node to the graph."""
        self._nodes.add(node)

    def add_edge(self, node1, node2, weight=1.0, bidirectional=True):
        """Adds an edge between node1 and node2. Adds the nodes to the graph first
        if they don't exist."""
        self.add_node(node1)
        self.add_node(node2)
        node1_edges = self._edges.get(node1, set())
        node1_edges.add(Edge(node1, node2, weight))
        self._edges[node1] = node1_edges
        if bidirectional:
            node2_edges = self._edges.get(node2, set())
            node2_edges.add(Edge(node2, node1, weight))
            self._edges[node2] = node2_edges

    def set_node_positions(self, positions):
        self.node_positions = positions

    def set_node_pos(self, node, pos):
        """Sets the (x,y) pos of the node, if it exists in the graph."""
        if not node in self:
            raise NodeNotInGraph(node)
        self.node_positions[node] = pos

    def get_node_pos(self, node):
        if not node in self:
            raise NodeNotInGraph(node)
        return self.node_positions[node]

    def node_edges(self, node):
        if not node in self:
            raise NodeNotInGraph(node)
        return self._edges.get(node, set())



import math
import random

# Implemented in assignment 3 by me

def eucl_dist(a, b):
    """Returns the euclidean distance between a and b."""
    # a is a tuple of (x,y) values of first point
    # b is a tuple of (x,y) values of second point
    hori_dist = a[0] - b[0]
    verti_dist = a[1] - b[1]

    return math.sqrt(hori_dist ** 2 + verti_dist ** 2)

##############################################
# --- Support functions for assignment 4 --- #
##############################################

def nearestSNode(graph, newNode):
    # returning tuple in nodeList that is closest to newNode
    nearestNodeInGraph = SearchNode((1,1))
    dist = eucl_dist((-10000,-10000),(10000,10000)) # huge distance
    for i in graph._nodes: # iterating through a set. i becomes a SEARCH NODE
        if (eucl_dist(i.state, newNode) < dist): #SEARCH NODE.state is a tuple
            nearestNodeInGraph = i # a search
            dist = eucl_dist(i.state, newNode)

    return nearestNodeInGraph, dist

def steerPath(firstNode,nextNode):
    # tuples as input - distance between them gives ish distance to move
    dist = eucl_dist(firstNode, nextNode)
    
    # avoiding errors when sampling node is too close
    if dist == 0:
        dist = 100000

    hori_dist = nextNode[0] - firstNode[0] #signed
    verti_dist = nextNode[1] - firstNode[1] #signed

    dist = dist

    # a new node that are closer to the next node - always smaller than the boundary-checked nextNode parameter
    # I chose to implement a slow but working solution -> normalize the distance to move. This could easily be changed
    return (firstNode[0] + hori_dist/dist, firstNode[1] + verti_dist/dist)

def obstacleIsInPath(firstNode,nextNode,env,radius):
    # a boolean for collision or not
    # this testing strategy is gathered from the RRT_examples from this assignment no. 4

    # Point from shapely
    start_pose = Point(firstNode).buffer(radius, resolution=3)
    end_pose = Point(nextNode).buffer(radius, resolution=3)

    # LineString from Shapely
    line = LineString([firstNode, nextNode])
    expanded_line = line.buffer(radius, resolution=3)

    for i, obs in enumerate(env.obstacles):
        # Check collisions between the expanded line and each obstacle
        if (expanded_line.intersects(obs)):
            return True

    return False

def goalReached(node,radius,end_region):
    # node is a tuple, radius the size of the robot
    # end_region is polygon of four tuples drawn clockwise: lower left, upper left, upper right, lower right

    # returns a boolean for node tuple + radius inside the region
    return end_region.contains(Point(node))

def plot_line_mine(ax, line):
    # wanted to draw lines in path more clearly. gathered format from environment.py
    x, y = line.xy
    ax.plot(x, y, color='black', linewidth=1, solid_capstyle='round', zorder=1)


def random_environment(bounds, start, radius, goal, n, size_limits=(0.5, 1.5)):
    minx, miny, maxx, maxy = bounds
    # print(bounds)
    edges = 4
    minl, maxl = size_limits
    env = Environment(None)
    obs = []
    start_pose = Point(start).buffer(radius, resolution=3)
    obi = 0
    while obi < n:
        r = np.random.uniform(low=0.0, high=1.0, size=2)
        xy = np.array([minx + (maxx-minx)*r[0], miny + (maxy-miny)*r[1]])
        
        angles = np.random.rand(edges)
        angles = angles*2*np.pi / np.sum(angles)
        for i in range(1,len(angles)):
            angles[i] = angles[i-1] + angles[i]
        angles = 2*np.pi * angles / angles[-1] 
        angles = angles + 2*np.pi*np.random.rand()
        lengths = 0.5*minl + (maxl-minl) * 0.5 * np.random.rand(edges)
        xx = xy[0] + np.array([l*np.cos(a) for a,l in zip(angles,lengths)])
        yy = xy[1] + np.array([l*np.sin(a) for a,l in zip(angles,lengths)])
        p = Polygon([(x,y) for x,y in zip(xx,yy)])
        if p.intersects(start_pose) or p.intersects(goal):
            continue
        else:
            obi = obi + 1
            obs.append(p)
#         coords = xy + [l*np.cos(a),l*np.sin(a) for a,l in zip(angles,lengths)]
    env.add_obstacles(obs)
    return env