import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as mpl
import quaternion

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
    x_right = (q * np.quaternion(0,drone_size[0]/2,0,0))* (q.conjugate())
    x_left = X - np.array([x_right.x,x_right.y,x_right.z])
    x_right = X + np.array([x_right.x,x_right.y,x_right.z])
    ax.plot3D([x_left[0],x_right[0]],[x_left[1],x_right[1]],[x_left[2],x_right[2]], color='red')

    y_front = (q * np.quaternion(0,0,drone_size[1]/2,0))* (q.conjugate())
    y_back = X - np.array([y_front.x,y_front.y,y_front.z])
    y_front = X + np.array([y_front.x,y_front.y,y_front.z])
    ax.plot3D([y_back[0],y_front[0]],[y_back[1],y_front[1]],[y_back[2],y_front[2]], color='green')
