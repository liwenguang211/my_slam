#ifndef CANNER_H_
#define CANNER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>
	
    int init_can();
    void end_can(int sock);
    int can_send(int sock,canid_t id,unsigned char* data);
    int can_receive(int sock,can_frame *fr2);
    
    

   
    
#endif