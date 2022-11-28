import numpy as np
#import networkx as nx
import arenaviz
import trajplan as tp
import quatfunc
import matplotlib.pyplot as plt

if __name__ == '__main__':

    # Arena and traj Definitions
    X = np.array([1,1,1],dtype = np.float64) # m,m,m
    X_wp = np.array([5,9,2.5],dtype = np.float64) # m,m,m
    X_goal = np.array([9,1,4],dtype = np.float64) # m,m,m
    arena_size = np.array([10,10,5],dtype = np.float64)  # m,m,m
    tf1 = 15 # sec
    tf2 = 15 # sec
    t_step = 0.01 # sec

    # Drone properties definition
    drone_size = np.array([1,1],dtype = np.float64) #m,m X and Y size, total
    mass = np.array([0.25]) #0.25 kg
    J = np.diag([0.1,0.1,0.1]) # Inertia matrix
    q = np.array([1,0,0,0])
    K_p = np.array([100,100,100])
    K_d = np.array([20,20,20])
    lda = 20
    k = 5

    # Physical Properties
    g = np.array([0,0,9.8]) #m/s^2, +ve as defined in eqs
    
    coeff = tp.trajpp(X,X_wp,X_goal,tf1,tf2)
    time = np.arange(0,tf1+tf2,t_step)

    r_des = np.zeros((3,time.size))
    for i in range(time.size):
        r_des[:,i] = tp.postime(coeff,time[i],tf1,tf2)
    
    r_desdot = np.gradient(r_des,t_step, axis = 1)
    r_des2dot = np.gradient(r_desdot,t_step, axis = 1)
    r_des3dot = np.gradient(r_des2dot,t_step, axis = 1)

    r_act = np.zeros((3,time.size))
    r_act[:,0] = X
    r_act_dot = np.zeros((3,time.size))
    r_act_2dot = np.zeros((3,time.size))
    r_err = r_des - r_act
    r_err_dot = np.zeros((3,time.size))

    T_des = np.zeros_like(r_des)
    T_des_unit = np.zeros_like(r_des)
    T_des_dot = np.zeros_like(r_des)
    F_des_unit = np.zeros_like(r_des)
    T_act = np.zeros_like(r_des)
    Mb_act = np.zeros_like(r_des)

    q_des = np.zeros((4,time.size))
    q_act = np.zeros((4,time.size))
    q_act_dot = np.zeros((4,time.size))
    q_act[:,0] = q
    q_err = np.zeros((4,time.size))
    q_err_dot = np.zeros((4,time.size))

    w_des = np.zeros_like(r_des)
    w_des_dot = np.zeros_like(r_des)
    w_act = np.zeros_like(r_des)
    w_err = np.zeros_like(r_des)
    w_act_dot = np.zeros_like(r_des)

    for i in range(time.size):
        T_des[2,i] = mass * np.linalg.norm(r_des2dot[:,i]+g)
        #print(mass*(r_des2dot[:,i]+g))
        norm = np.linalg.norm(T_des[:,i])
        if norm == 0:
            T_des_unit[:,i] = T_des[:,i]
        else:
            T_des_unit[:,i] = T_des[:,i]/norm
        if i>0:
            T_des_dot[:,i] = (T_des[:,i]-T_des[:,i-1])/t_step
        
        F_des_unit[:,i] = mass * (r_des2dot[:,i]+g)
        norm = np.linalg.norm(F_des_unit[:,i])
        if norm == 0:
            F_des_unit[:,i] = F_des_unit[:,i]
        else:
            F_des_unit[:,i] = F_des_unit[:,i]/norm
        
        q_des[:,i] = (1/np.sqrt(2*(np.array([1])+np.dot(T_des_unit[:,i],F_des_unit[:,i])))) * np.concatenate((np.array([1])+np.dot(T_des_unit[:,i],F_des_unit[:,i]),np.cross(T_des_unit[:,i],F_des_unit[:,i])))
        #q_des[:,i] = quatfunc.quat_norm(q_des[:,i]) #No need as already unit
        
        w_temp = ( mass * quatfunc.quat_mul3( quatfunc.quat_conj(q_des[:,i]) , np.concatenate(([0],r_des3dot[:,i])) , q_des[:,i] )[1:4]  - T_des_dot[:,i])/ T_des[2,i]
        w_des[:,i] = np.array([-1*w_temp[1],w_temp[0],0])
        if i>0:
            w_des_dot[:,i] = (w_des[:,i]-w_des[:,i-1])/t_step

        ## Control Loop
        
        q_err[:,i] = quatfunc.quat_mul(quatfunc.quat_conj(q_des[:,i]),q_act[:,i])
        if i>0:
            q_err_dot[:,i] = (q_err[:,i]-q_err[:,i-1])/t_step
        w_err[:,i] = w_act[:,i] - quatfunc.quat_mul3(quatfunc.quat_conj(q_err[:,i]), np.concatenate(([0],w_des[:,i])),q_err[:,i])[1:4]
        r_err[:,i] = r_act[:,i] - r_des[:,i]
        if i>0:
            r_err_dot[:,i] = (r_err[:,i]-r_err[:,i-1])/t_step

        
        s = w_err[:,1] + lda * np.sign(q_err[0,i])*q_err[1:4,i]
        Mb_act[:,i] = np.cross(w_act[:,i], J @ w_act[:,i]) + J @ w_des_dot[:,i] - lda * np.sign(q_err[0,i]) * J  @ q_err_dot[1:4,i] - k * J @ s
        T_act[:,i] = quatfunc.quat_mul3(quatfunc.quat_conj(q_act[:,i]), np.concatenate(([0],mass*(r_des2dot[:,i]+g-K_p*r_err[:,i]-K_d*r_err_dot[:,i]))) , q_act[:,i])[1:4] #T_act[:,i]
        #temp = np.concatenate(([0],mass*(r_des2dot[:,i]+g-K_p*r_err[:,i]-K_d*r_err_dot[:,i])))
        ## Dynamics
        if (i==(time.size-1)):
            break
        #r_act_2dot[:,i+1] = -1*g +  r_des2dot[:,i]+g  - K_p*r_err[:,i]-K_d*r_err_dot[:,i]
        r_act_2dot[:,i+1] = -1*g + 0.95* (r_des2dot[:,i]+g  - K_p*r_err[:,i]-K_d*r_err_dot[:,i])
        #r_act_2dot[:,i+1] = -1*g + (quatfunc.quat_mul3(q_act[:,i], np.concatenate(([0],1*T_act[:,i])),quatfunc.quat_conj(q_act[:,i]))[1:4])/mass

        r_act_dot[:,i+1] = r_act_2dot[:,i+1]*t_step + r_act_dot[:,i]
        r_act[:,i+1] = r_act_dot[:,i+1]*t_step + r_act[:,i]
        q_act_dot[:,i+1] = 0.5*quatfunc.quat_mul( q_act[:,i],np.concatenate(([0],w_act[:,i])) )
        q_act[:,i+1] = q_act_dot[:,i+1]*t_step + q_act[:,i]
        w_act_dot[:,i+1] = np.linalg.inv(J) @ ( np.cross(-1*w_act[:,i], J @ w_act[:,i]) + 0.95*Mb_act[:,i])
        w_act[:,i+1] = w_act_dot[:,i+1]*t_step + w_act[:,i]
    
    # fig = plt.figure(figsize=(10, 10))
    # ax = plt.axes(projection = '3d')
    # for i in range(time.size):
    #     #if i%10==0:
    #     arenaviz.plot_state(r_act[:,i],q_act[:,i],arena_size,drone_size,i,fig,ax)

    # Uncomment to visualise x axis pos, vel, accn, jerk
    fig , axs = plt.subplots(4,1)
    axs[0].plot(time,np.linalg.norm(r_err,axis = 0))
    axs[1].plot(time,np.sum(q_err,axis = 0)-1)
    axs[2].plot(time,np.linalg.norm(Mb_act,axis = 0))
    axs[3].plot(time,np.sum(T_act,axis = 0))
    plt.show()
    ##arenaviz.plot_state(X,q,arena_size,drone_size)