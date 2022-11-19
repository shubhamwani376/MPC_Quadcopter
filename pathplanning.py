# # import matplotlib.pyplot as plt
# # import numpy as np
# # import networkx as nx

# # RRT* Pseudo Code
# # Rad = r
# # G(V,E) //Graph containing edges and vertices
# # For itr in range(0…n)
# #     Xnew = RandomPosition()
# #     If Obstacle(Xnew) == True, try again
# #     Xnearest = Nearest(G(V,E),Xnew)
# #     Cost(Xnew) = Distance(Xnew,Xnearest)
# #     Xbest,Xneighbors = findNeighbors(G(V,E),Xnew,Rad)
# #     Link = Chain(Xnew,Xbest)
# #     For x’ in Xneighbors
# #         If Cost(Xnew) + Distance(Xnew,x’) < Cost(x’)
# #             Cost(x’) = Cost(Xnew)+Distance(Xnew,x’)
# #             Parent(x’) = Xnew
# #             G += {Xnew,x’}
# #     G += Link 
# # Return G 
# # https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378

# # def rrt_star(X,X_goal,arena_size,thresh,path_thresh):
# #     G = nx.Graph()
# #     G.add_node(tuple(X))
# #     for i in list(range(10000)):
# #         X_rand = np.random.Generator.uniform(low = size=[3,1])*arena_size
# #         print(X_rand)
# #         path = np.array([])

#     #G.add_edge(2,3,weight = 0.9)

#  #   return(path) # which will be a 3*m array

# #def djikstras(G)
