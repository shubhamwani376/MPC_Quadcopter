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
    J = np.diag([0.01,0.01,0.01]) # Inertia matrix
    #q = np.array([1,0,0,0])
    q = np.quaternion(1,0,0,0)
    #r = quaternion.as_float_array(q)
    #s = quaternion.as_quat_array(r)

    # Physical Properties
    g = np.array([0,0,-9.8]) #m/s^2
    
    coeff = tp.trajpp(X,X_wp,X_goal,tf1,tf2)
    time = np.arange(0,tf1+tf2,t_step)
    r_des = np.zeros((3,time.size))
    for i in range(time.size):
        r_des[:,i] = tp.postime(coeff,time[i],tf1,tf2)
    r_des1dot = np.gradient(r_des,t_step, axis = 1)
    r_des2dot = np.gradient(r_des1dot,t_step, axis = 1)
    r_des3dot = np.gradient(r_des2dot,t_step, axis = 1)

    T_des = np.zeros_like(r_des)
    T_des_unit = np.zeros_like(r_des)
    F_des_unit = np.zeros_like(r_des)
    q_des = np.zeros((4,time.size))
    w_des = np.zeros_like(r_des)
    for i in range(time.size):
        T_des[2,i] = mass * np.linalg.norm(r_des[:,i]+g)
        norm = np.linalg.norm(T_des[:,i])
        if norm == 0:
            T_des_unit[:,i] = T_des[:,i]
        else:
            T_des_unit[:,i] = T_des[:,i]/norm
        
        F_des_unit[:,i] = mass * (r_des[:,i]+g)
        norm = np.linalg.norm(F_des_unit[:,i])
        if norm == 0:
            F_des_unit[:,i] = F_des_unit[:,i]
        else:
            F_des_unit[:,i] = F_des_unit[:,i]/norm
        
        #q_des[:,i] = (1/np.sqrt(2*(np.array([1])+np.dot(T_des_unit[:,i],F_des_unit[:,i])))) * np.concatenate(np.array([1])+np.dot(T_des_unit[:,i],F_des_unit[:,i]),np.cross(T_des_unit[:,i],F_des_unit[:,i]))
        q_des[:,i] = (1/np.sqrt(2*(1+np.dot(T_des_unit[:,i],F_des_unit[:,i]))))*
        #q_des[:,i] = np.concatenate(1+np.dot(T_des_unit[:,i],F_des_unit[:,i]),np.cross(T_des_unit[:,i],F_des_unit[:,i]))
        #p = np.array([1])+np.dot(T_des_unit[:,i],F_des_unit[:,i])
        #q = np.cross(T_des_unit[:,i],F_des_unit[:,i])
        #r = np.concatenate(p,q.T)
        #w_des[0:2,i] = mass * np.linalg.norm(r_des[:,i]+g)
        
    #print(T_des_unit,F_des_unit,q_des,w_des)
    print(p,q)


    # Uncomment to visualise x axis pos, vel, accn, jerk
    # fig , axs = plt.subplots(4,1)
    # axs[0].plot(time,r_des[0,:])
    # axs[1].plot(time,r_des1dot[0,:])
    # axs[2].plot(time,r_des2dot[0,:])
    # axs[3].plot(time,r_des3dot[0,:])
    # plt.show()

    #arenaviz.plot_state(X,q,arena_size,drone_size)