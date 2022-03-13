import numpy as np
import os
import matplotlib.pyplot as plt


def main():
    f=open('/home/cuichenxi/mediapipe/mediapipe/examples/desktop/visiualize/data.txt','r',encoding='utf-8')
    points=[]
    for line in f:
        line=line.strip()
        line=line.split()
        points.append([float(line[0]),float(line[1]),float(line[2])])
    #print(points)
    kp3d=np.array(points)
    print(kp3d[[0,1,3,5]].T)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(*kp3d[[16,14,12,11,13,15]].T)
    ax.plot(*kp3d[[12,24,23,11,12]].T)
    ax.plot(*kp3d[[10,9]].T)
    ax.plot(*kp3d[[8,6,5,4,0,1,2,3,7]].T)
    ax.plot(*kp3d[[12,24,26,28]].T)
    ax.plot(*kp3d[[11,23,25,27]].T)
    plt.show()


if __name__=="__main__":
    main()


