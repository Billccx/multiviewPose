import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

x=[]
y=[]
z=[]
cnt=0


f=open("/home/cuichenxi/mediapipe/mediapipe/examples/desktop/ceres_test/result/1.txt",'r',encoding='utf-8')
for line in f:
    cnt+=1
    line=line.strip()
    line=line.split(' ')
    x.append(float(line[0]))
    y.append(float(line[1]))
    z.append(float(line[2]))
    if(cnt==13): break

print(x)
print(y)
print(z)

fig = plt.figure()
ax = fig.add_axes((0.1,0.1,0.8,0.8), projection='3d')
ax.plot(x, y, z, c='red', marker='o')

# ax3d = Axes3D(fig)
# ax3d.scatter(x,y,z,c="b",marker="*")

plt.show()

