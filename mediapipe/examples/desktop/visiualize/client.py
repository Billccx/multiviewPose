import socket
import time

port = 31200


def main():
    s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    host = socket.gethostname()

    sendlist=[]
    for i in range(1,26):
        f=open('/home/cuichenxi/mediapipe/mediapipe/examples/desktop/visiualize/data/'+str(i)+'.txt','r',encoding='utf-8')
        sendStr=""
        for index,line in enumerate(f):
            if(index<69):
                continue
            line=line.strip()
            line=line.split()
            sendStr+=line[0]+' '+line[1]+' '+line[2]+' '
        sendlist.append(sendStr)


    cnt=0
    while(cnt<1000):
        sendData = sendlist[cnt%25]
        s.sendto(sendData.encode('utf-8'),(host,port))
        time.sleep(0.1)
        cnt+=1
    s.close()

if __name__=='__main__':
    main()