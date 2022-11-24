import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as mpl
import quatfunc

def plot_state(X,q,arena_size,drone_size):
    fig = plt.figure()
    ax = plt.axes(projection = '3d')
    ax.plot3D(X[0],X[1],X[2],'bo')
    ax.set_xlim(0, arena_size[0])
    ax.set_ylim(0, arena_size[1])
    ax.set_zlim(0, arena_size[2])
    ax.bar3d([4.5],[0],[0],[1],[8],[5],shade = 'true')
    plot_orientation(X,q,drone_size,ax)
    ax.view_init(elev=ax.elev, azim=ax.azim-90)
    plt.show()

def plot_orientation(X,q,drone_size,ax):
    x_right = quatfunc.quat_mul( quatfunc.quat_mul(q , np.array([0,drone_size[0]/2,0,0])) , quatfunc.quat_conj(q) )
    x_left = X - np.array([x_right[1],x_right[2],x_right[3]])
    x_right = X + np.array([x_right[1],x_right[2],x_right[3]])
    ax.plot3D([x_left[0],x_right[0]],[x_left[1],x_right[1]],[x_left[2],x_right[2]], color='red')

    y_front = quatfunc.quat_mul( quatfunc.quat_mul(q , np.array([0,0,drone_size[1]/2,0])) , quatfunc.quat_conj(q) )
    y_back = X - np.array([y_front[1],y_front[2],y_front[3]])
    y_front = X + np.array([y_front[1],y_front[2],y_front[3]])
    ax.plot3D([y_back[0],y_front[0]],[y_back[1],y_front[1]],[y_back[2],y_front[2]], color='green')



# # Code to check this function
# X = np.array([1,1,1],dtype = np.float64) # m,m,m
# X_wp = np.array([5,9,2.5],dtype = np.float64) # m,m,m
# X_goal = np.array([9,1,4],dtype = np.float64) # m,m,m
# arena_size = np.array([10,10,5],dtype = np.float64)  # m,m,m
# tf1 = 30 # sec
# tf2 = 30 # sec
# t_step = 0.01 # sec
# drone_size = np.array([1,1],dtype = np.float64)

# # Drone properties definition
# drone_size = np.array([1,1],dtype = np.float64) #m,m X and Y size, total 
# q = np.array([1,0,0,0])
# q2 = np.array([np.sqrt(2)/2,0,0,-1*np.sqrt(2)/2])
# plot_state(X,q,arena_size,drone_size)