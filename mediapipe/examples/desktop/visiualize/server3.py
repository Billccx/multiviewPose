import numpy as np
import socket
import os
import time
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation

port = 31201
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   
s.setblocking(0)      
host = socket.gethostname()
s.bind((host,port))
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title('3D Pose')


def plot(kp3d):
    plt.cla()
    ax.plot(*kp3d[[16,14,12,11,13,15]].T)
    ax.plot(*kp3d[[12,24,23,11,12]].T)
    ax.plot(*kp3d[[10,9]].T)
    ax.plot(*kp3d[[8,6,5,4,0,1,2,3,7]].T)
    ax.plot(*kp3d[[12,24,26,28]].T)
    ax.plot(*kp3d[[11,23,25,27]].T)



def update_graph(num):
    try:
        kps=s.recvfrom(2048)
        print(kps)
        kps=kps.strip()
        kps=kps[:-1]
        kps=kps.split(',')


        kp3d=[]
        for item in kps:
            kp3d.append(float(item))
        kp3d=np.array(kp3d)
        kp3d=kp3d.reshape((33,3))
        print(kp3d[0])
        plot(kp3d)
    except:
        #print("receive nothing")
        pass




def main():
    print(host)

    kp3d=np.random.rand(33,3)
    plot(kp3d)

    ani = matplotlib.animation.FuncAnimation(fig, update_graph,  
                               interval=10, blit=False)

    plt.show()


    


if __name__=='__main__':
    main()