#include "canner.h"

int init_can()
{
    int ret, s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    bool can_enable = true;

    //system("sudo ifconfig can0 down");
    //system("sudo ip link set can0 type can bitrate 1000000");
    //system("sudo ifconfig can0 up");
    printf("this is a can send demo\r\n");

    //1.Create socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
    {
        perror("socket PF_CAN failed");
        can_enable = false;
    }

    //2.Specify can0 device
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(s, SIOCGIFINDEX, &ifr);
    if (ret < 0)
    {
        perror("ioctl failed");
        can_enable = false;
    }

    //3.Bind the socket to can0
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0)
    {
        perror("bind failed");
        can_enable = false;
    }

    //4.Disable filtering rules, do not receive packets, only send
	struct can_filter rfilter[4];
	rfilter[0].can_id = 0x185;
	rfilter[0].can_mask = CAN_SFF_MASK;
	rfilter[1].can_id = 0x186;
	rfilter[1].can_mask = CAN_SFF_MASK;
	rfilter[2].can_id = 0x187;
	rfilter[2].can_mask = CAN_SFF_MASK;
	rfilter[3].can_id = 0x188;
	rfilter[3].can_mask = CAN_SFF_MASK;

    	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
	
    if (!can_enable)
    {
        printf("can device not initialize!\n");
        can_enable = true;
        return -1;
    }
	printf("can device initialize success=%d!\n",s);
    return s;
}

void end_can(int sock)
{
    close(sock);
    //system("sudo ifconfig can0 down");
}

int can_send(int sock, canid_t id, unsigned char *data)
{
    usleep(1000);
        int nbytes;
    struct can_frame fr;
    memset(&fr, 0, sizeof(struct can_frame));
    fr.can_id = id;
    fr.can_dlc = 8;

    for (int i = 0; i < fr.can_dlc; i++)
    {
        fr.data[i] = data[i];
    }
    nbytes = write(sock, &fr, sizeof(fr));
    
    if (nbytes < 0)
        return -1;
    else
        return 0;
}

int can_receive(int sock,can_frame *fr2)
{
	//printf("niubi66666666666666666666666,%d\n",sock);
    int nbytes;
     nbytes = read(sock, fr2, sizeof(can_frame));
	//printf("niubi99999999999999999999999\n");

       // printf("%x\n",fr2->can_id);
	
    if (nbytes < 0)
        return -1;
    else
        return 0;
}