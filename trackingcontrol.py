import numpy as np
#import networkx as nx
import arenaviz
import trajplan as tp
import quaternion
import matplotlib.pyplot as plt

if __name__ == '__main__':

    # Arena and traj Definitions
    X = np.array([1,1,1],dtype = np.float64) # m,m,m
    X_wp = np.array([5,9,2.5],dtype = np.float64) # m,m,m
    X_goal = np.array([9,1,4],dtype = np.float64) # m,m,m
    arena_size = np.array([10,10,5],dtype = np.float64)  # m,m,m
    tf1 = 30 # sec
    tf2 = 30 # sec
    t_step = 0.01 # sec

    # Drone properties definition
    drone_size = np.array([1,1],dtype = np.float64) #m,m X and Y size, total 
    mass = np.array([0.25]) #0.25 kg
    #q = np.array([1,0,0,0])
    q = np.quaternion(1,0,0,0)

    # Physical Properties
    g = np.array([0,0,9.8]) #m/s^2

    coeff = tp.trajpp(X,X_wp,X_goal,tf1,tf2)
    time = np.arange(0,tf1+tf2,t_step)
    r_des = np.zeros((3,time.size))
    for i in range(time.size):
        r_des[:,i] = tp.postime(coeff,time[i],tf1,tf2)
    r_des1dot = np.gradient(r_des,t_step, axis = 1)
    r_des2dot = np.gradient(r_des1dot,t_step, axis = 1)
    r_des3dot = np.gradient(r_des2dot,t_step, axis = 1)

    # T_des = np.zeros_like(r_des)
    # for i in range(time.size):
    #     T_des[2,i] = mass * np.linalg.norm()


    # Uncomment to visualise x axis pos, vel, accn, jerk
    # fig , axs = plt.subplots(4,1)
    # axs[0].plot(time,r_des[0,:])
    # axs[1].plot(time,r_des1dot[0,:])
    # axs[2].plot(time,r_des2dot[0,:])
    # axs[3].plot(time,r_des3dot[0,:])
    # plt.show()

    arenaviz.plot_state(X,q,arena_size,drone_size)