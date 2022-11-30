import numpy as np

def trajpp(X,X_wp,X_goal,tf1,tf2):
    coeff = np.zeros((6,6))
    for i in range(3):
        coeff[:,i] = polycoeff(X[i],X_wp[i],tf1)
    for j in range(3):
        coeff[:,j+3] = polycoeff(X_wp[j],X_goal[j],tf2)
    np.set_printoptions(precision=3)
    return coeff

def polycoeff(pos_start,pos_end,tf):
    bound = np.array([pos_start,pos_end,0,0,0,0],dtype = float)
    mat = np.array([[0,0,0,0,0,1],[tf**5,tf**4,tf**3,tf**2,tf**1,1],[0,0,0,0,1,0],[5*tf**4,4*tf**3,3*tf**2,2*tf,1,0],[0,0,0,2,0,0],[20*tf**3,12*tf**2,6*tf,2,0,0]],dtype = float)
    coeff_sub = np.matmul(np.linalg.inv(mat) , bound)
    return coeff_sub

def postime(coeff,time,tf1,tf2):
    #pos = np.array([0,0,0],dtype = float) # m,m,m
    if time <= tf1:
        # 1st wala poly
        timearray = np.array([time**5,time**4,time**3,time**2,time**1,1])
        pos = np.multiply(timearray,coeff[:,0:3].T)
        pos = np.sum([pos],axis = 2)
    else:
        time = time - tf1
        timearray = np.array([time**5,time**4,time**3,time**2,time**1,1])
        pos = np.multiply(timearray,coeff[:,3:6].T)
        pos = np.sum([pos],axis = 2)
    return pos #x,y,z as np array