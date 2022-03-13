#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <iostream>
#include <string>
using namespace std;
int main(){
    int udp_socket; 
    const int port=31201;
    char buffer[2048];
    struct sockaddr_in servaddr;
    if ( (udp_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(buffer,0,sizeof(buffer));
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    int cnt=0;
    while(cnt<1000){
        //cout<<cnt<<endl;

        memset(buffer,0,sizeof(buffer));
        int p=0;
        for(int i=0;i<33;i++){
            p+=sprintf(buffer+p,"%d ",cnt);
            p+=sprintf(buffer+p,"%d ",cnt);
            p+=sprintf(buffer+p,"%d ",cnt);
        }
        //printf("%s\n",buffer);
        printf("%d\n",strlen(buffer));
        usleep(10000);
        sendto(udp_socket, (const char *)buffer, strlen(buffer),MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
        cnt++;
    }
}