import numpy as np
import socket
import os
import time
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title('3D Pose')


f=open("/home/cuichenxi/mediapipe/mediapipe/examples/desktop/ceres_test/dt/r.txt",'r',encoding='utf-8')

for line in f:
    line=line.strip()
    line=line.split()
    ax.scatter(np.float128(line[0]),float(line[2]),-float(line[1]))

plt.show()