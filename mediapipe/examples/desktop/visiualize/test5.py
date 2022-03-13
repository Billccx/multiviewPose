#!/usr/bin/env python 
# -*- coding:utf-8 -*-


from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
#初始化图例
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#设置X,Y的取值
r = [2,2]
p = np.linspace(0, 2*np.pi, 64)
R, P = np.meshgrid(r, p)
X, Y = R * np.cos(P), R * np.sin(P)
r1=[1,0]
#设置Z的范围
ax.set_zlim(0, 10)
try:
    while True:
        #清除原有图像
        plt.cla()
        p1 = np.random.randint(10, size=64)
        R1, P1 = np.meshgrid(r1, p1)
        #设置Z值
        Z = R1*P1
        #画3D图
        ax.plot_surface(X, Y, Z, cmap=plt.cm.plasma)
        #通过暂停和清除来不断更新图像，形成动图
        plt.pause(0.5)
except:
    pass


