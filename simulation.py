from matplotlib import pyplot as plt
import numpy as np
import networkx as nx
import pathplanning as path
import plot3denv as p3denv

if __name__ == '__main__':
    # Total Arena 10x10x5
    X = np.array([1,1,1])
    X_goal = np.array([9,9,4])
    arena_size = np.array([10,10,5])
    X_goal_threshold = 0.1 # metre, sphere
    path_threshold = np.array([0.1])
    path_threshold = np.array([0.1])

    # Performing RRT* on env and getting the waypoints
    path_rrt = np.array([])
    path_rrt = path.rrt_star(X, X_goal, arena_size,  X_goal_threshold, path_threshold)

    # Visualise the path
    #p3denv.



