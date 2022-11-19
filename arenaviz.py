import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as mpl

def plotstate(X,q,arena_size,dronesize)
    fig = plt.figure()
    ax = plt.axes(projection = '3d')
    ax.plot3D([X])
    plt.show()