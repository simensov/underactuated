# Useful imports
from __future__ import division
from matplotlib import pyplot as plt
import numpy as np
import yaml
from shapely.geometry import Point, Polygon, LineString, box
from environment import Environment, plot_environment, plot_line, plot_poly
from check_path import check_path

from support import *

plotting = False

def rrt(bounds, environment, start_pose, radius, end_region):
    '''
    - bounds: (minx, miny, maxx, maxy) tuple over region
    - environment: instance of the Environment class that describes the placement of the obstacles in the scene
    - start_pose: start_pose = (x,y) is a tuple indicating the starting position of the robot
    - radius: radius is the radius of the robot (used for collision checking)
    - end_region: end_region is a shapely Polygon that describes the region that the robot needs to reach
    '''

    # Adding tuples to nodes -> represent all nodes expanded by tree
    nodes = [start_pose]

    # Creating graph of SearchNodes from the tuples to represent the tree with parents. Used to order and
    graph = Graph()
    graph.add_node(SearchNode(start_pose))
    goalPath = Path(SearchNode(start_pose)) # for initialization in case of no path found
    
    # Draw the environment (with its obstacles) and with the start node + end region
    if plotting:
        ax = plot_environment(environment,bounds)
        plot_poly(ax, Point(start_pose).buffer(radius, resolution=3),'blue')
        plot_poly(ax, end_region,'red', alpha=0.2)
    
    for i in range(10000):  # random high number
        # this range must be evaluated according to size of problem. RRT do not ensure any solution paths
        
        # sample a random node inside bounds (random number between xmin/max and ymin/max). all bound-"checking" is donw here: makes sure we don't have to do ant checks later
        rand_xval = random.uniform(bounds[0],bounds[2])
        rand_yval = random.uniform(bounds[1],bounds[3])
        node_rand = (rand_xval,rand_yval)
        
        '''
        --- For every x'th iteration - aim towards goal. A good choice varies, and depends on the step length that I decided in steerpath(). Not having this makes total number of nodes blow up. I have kept choices of when to goal bias and the travel distance in steering function to perform well in large environments. TODO: This number depends on complexity of space. Easy space: lower number
        --- Then, find out which node in our list that is the nearest to the random node. Returns a searchNode and a float distance.
        --- Steer towards the new node -> correct parents and add cost
        --- Check if the new steered node is in bounds.
        --- If not visited before (avoid cycle) and no obstacles are in the path: add to tree
        '''
        
        sampling_rate = 12
        if not(i % sampling_rate):
            node_rand = end_region.centroid.coords[0]

        node_nearest, node_dist = nearestSNode(graph, node_rand) 

        ## TODO: this steering function should somehow consider dynamics
        steered_node = steerPath(node_nearest.state, node_rand)
        
        if not (bounds[0] < steered_node[0] < bounds[2]) or not (bounds[1] < steered_node[1] < bounds[3]):
            continue # sometimes not checking for this made the path go out of bounds

        node_steered = SearchNode(steered_node,node_nearest,node_nearest.cost+node_dist)
        
        if node_steered.state not in nodes:
            if not obstacleIsInPath(node_nearest.state, node_steered.state, environment,radius):

                # Keep track of total number of nodes in tree            
                # Add edge (also adds new nodes to graph) and weight
                # Plot non-colliding edge to show all searched nodes
                nodes.append(node_steered.state)
                graph.add_edge(node_nearest,node_steered,node_dist)

                if plotting:
                    line = LineString([node_nearest.state,node_steered.state])
                    plot_line_mine(ax, line)

            else:
                continue # Avoid goal check if collision is found

        # Check last addition for goal state
        # TODO: ADDED EXTRA SAFETY IN GOAL CHECK
        if goalReached(node_steered.state, 1.5 * radius,end_region):  
            goalPath = Path(node_steered)
            break # break the while loop when solution is found!

    # No. of nodes in total - No. of nodes in sol path - Sol path length
    noOfTotalNodes = len(nodes); 
    noOfNodesInSol = len(goalPath.path); 
    pathLength = goalPath.cost

    if plotting:
        for i in range(noOfNodesInSol-1):
            # Draw goal path
            line = LineString([goalPath.path[i], goalPath.path[i+1]])
            plot_line_mine(ax, line)
            
            # Draw the expanded line
            expanded_line = line.buffer(radius, resolution=3)
            plot_poly(ax, expanded_line, 'green', alpha=0.2)
            
            # plotting last node in goalPath and setting title to format in task
            plot_poly(ax, Point(goalPath.path[-1]).buffer(radius, resolution=3),'blue')
            titleString = "Nodes total / in solution path: %s/ %s \nPath length: %0.3f"
            ax.set_title(titleString % (noOfTotalNodes,noOfNodesInSol,pathLength))
        
    return goalPath.path

def plotGoalPath(path, radius, ax, end_region):
    '''
    path:   list of (x,y)-tuples 
    ax:     matplotlib.axes object

    Purpose: is meant to plot RRT tree on drone
    '''
    plot_poly(ax, end_region,'green', alpha=0.3)

    plot_poly(ax, Point(path[0]).buffer(radius/2, resolution=3),'blue',alpha=0.3)

    for i in range(0,len(path)-1):
        line = LineString([path[i], path[i+1]])
        # plot_line_mine(ax, line)
        
        expanded_line = line.buffer(radius, resolution=3)
        plot_poly(ax, expanded_line, 'green', alpha=0.025)
        
    # plotting last node in goalPath and setting title to format in task
    # plot_poly(ax, Point(path[-1]).buffer(radius/2, resolution=3),'blue')