import numpy as np
# from matplotlib import pyplot as plt
# import networkx as nx
# import plot3denv as p3denv
import trajplan as tp

if __name__ == '__main__':
    X = np.array([1,1,1],dtype = float) # m,m,m
    X_wp = np.array([5,9,2.5],dtype = float) # m,m,m
    X_goal = np.array([9,1,4],dtype = float) # m,m,m
    arena_size = np.array([10,10,5])  # m,m,m
    tf1 = 30 # sec
    tf2 = 30 # sec
    time = 1
    coeff = tp.trajpp(X,X_wp,X_goal,tf1,tf2)
    goal = tp.postime(coeff,time,tf1,tf2)
    print(goal)
    #time = np.linspace

    #for t in np.linspace



    # X_goal_threshold = 0.1 # metre, sphere
    # path_threshold = np.array([0.1])
    # path_threshold = np.array([0.1])

    # Performing RRT* on env and getting the waypoints
    # path_rrt = np.array([])
    # path_rrt = path.rrt_star(X, X_goal, arena_size,  X_goal_threshold, path_threshold)

    # Visualise the path
    #p3denv.