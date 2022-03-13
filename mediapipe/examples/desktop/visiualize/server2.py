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
s.bind(("127.0.0.1",port))


def main():
    while(1):
        try:
            data=s.recvfrom(2048)
            print(data)
        except:
            #print("wdcd")
            pass


if __name__=='__main__':
    main()