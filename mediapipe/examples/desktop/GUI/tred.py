from threading import Thread
import os
import time


def cmd(strs):
    for str in strs:
        os.system(str)
    time.sleep(10)
    os.system()


def main():
    strs=['cd ~/mediapipe/','GLOG_logtostderr=1 bazel-bin/mediapipe/examples/desktop/mypose7/mypose7_cpu --calculator_graph_config_file=mediapipe/examples/desktop/mypose7/mypose_tracking8.pbtxt']
    t=Thread(target=cmd,args=(strs,))
    t.start()
    time.sleep(5)





if __name__=='__main__':
    main()
