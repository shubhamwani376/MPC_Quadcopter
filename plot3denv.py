import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
 
 
# Create axis
axes = [10, 10, 5]
 
# Create Data
data = np.ones(axes, dtype=bool)
 
# Control Transparency
alpha = 0.9
 
# Control colour
colors = np.empty(axes + [4], dtype=np.float32)
 
colors[0] = [1, 0, 0, alpha]  # red
colors[1] = [0, 1, 0, alpha]  # green
colors[2] = [0, 0, 1, alpha]  # blue
colors[3] = [1, 1, 0, alpha]  # yellow
colors[4] = [1, 1, 1, alpha]  # grey
colors[5] = [1, 1, 1, alpha]  # grey
colors[6] = [1, 1, 1, alpha]  # grey
colors[7] = [1, 1, 1, alpha]  # grey
colors[8] = [1, 1, 1, alpha]  # grey
colors[9] = [1, 1, 1, alpha]  # grey
 
# Plot figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
data[ 0] = False
data[-1] = False
data[-2] = False
data[-3] = False
data[-4] = False
data[-5] = True
data[-6] = False
data[-7] = False
data[-8] = False
data[-9] = False
 
# Voxels is used to customizations of
# the sizes, positions and colors.
ax.voxels(data, facecolors=colors)
plt.show()
