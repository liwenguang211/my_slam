#include <stdlib.h>  
#include <stdio.h> 
#include <string.h>  
#include <errno.h>  
#include <sys/types.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <unistd.h>
#include <arpa/inet.h>  
#include <sys/time.h>

#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include "imu.h"
#include "common/time.h"
#include "common/configuration.h"
FILE *fp_2;
IMUModule::IMUModule() {
    pthread_mutex_init(&mutex, nullptr);
    pthread_create(&imu_thread, NULL, thread_function, this);
	fp_2=fopen("imu1.txt","w");
};

IMUModule::~IMUModule() {
    thread_running = false;
    close_imu();
    pthread_mutex_destroy(&mutex);
    pthread_join(imu_thread, NULL);
};

speed_t IMUModule::getBaudrate(int baudrate)
{
	switch(baudrate) {
	case 0: return B0;
	case 50: return B50;
	case 75: return B75;
	case 110: return B110;
	case 134: return B134;
	case 150: return B150;
	case 200: return B200;
	case 300: return B300;
	case 600: return B600;
	case 1200: return B1200;
	case 1800: return B1800;
	case 2400: return B2400;
	case 4800: return B4800;
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 115200: return B115200;
	case 230400: return B230400;
	case 460800: return B460800;
	case 500000: return B500000;
	case 576000: return B576000;
	case 921600: return B921600;
	case 1000000: return B1000000;
	case 1152000: return B1152000;
	case 1500000: return B1500000;
	case 2000000: return B2000000;
	case 2500000: return B2500000;
	case 3000000: return B3000000;
	case 3500000: return B3500000;
	case 4000000: return B4000000;
	default: return -1;
	}
}

int IMUModule::open_tty(char const *path, int baudrate, int flags)
{
	int fd;
	speed_t speed;

	if ((speed = getBaudrate(baudrate)) == -1) {
		return -1;
	}
	if ((fd = open(path, O_RDWR | flags)) == -1){
		return -1;
	}

	struct termios cfg;
	if (tcgetattr(fd, &cfg)){
		close(fd);
		return -1;
	}
	cfmakeraw(&cfg);
	cfsetispeed(&cfg, speed);
	cfsetospeed(&cfg, speed);
	cfg.c_cc[VMIN] = 1;
	cfg.c_cc[VTIME] = 1;
	if (tcsetattr(fd, TCSANOW, &cfg)){
		close(fd);
		return -1;
	}
	
	printf("imu :open %s ok,fd=%d\n",path,fd); 
	return fd;
}
int IMUModule::toBCD(int x){
	return ((x%256)/16)*10+(x%16);
}
#define PI 3.1415926
int IMUModule::check_CS(char* p,int len){
	int sum=0;
	for(int j=0;j<len-1;j++){
		sum+=p[j];
	}
	if((sum%0x100)==p[len-1])
		return 1;
	else
		return 0;
}

#define ACCEL_G	9.80665 
void IMUModule::handle_frame(char* p,int len,IMUModule* ptr){
//	int checksum;
	long timestamp;
	int addr=p[1];
	int cmd=p[2];
	
	z_angular_velocity=(float)((p[3]&0x10)==0x10?
	(0-toBCD(p[3]&0x0f)*10000-toBCD(p[4])*100-toBCD(p[5])):
	(toBCD(p[3]&0x0f)*10000+toBCD(p[4])*100+toBCD(p[5])))/100;
	
	//m/s2
	forward_linear_accel=(float)((p[6]&0x10)==0x10?
	(0-toBCD(p[6]&0x0f)*10000-toBCD(p[7])*100-toBCD(p[8])):
	(toBCD(p[6]&0x0f)*10000+toBCD(p[7])*100+toBCD(p[8])))/1000;
	
	z_angle=(float)((p[9]&0x10)==0x10?
	(0-toBCD(p[9]&0x0f)*10000-toBCD(p[10])*100-toBCD(p[11])):
	(toBCD(p[9]&0x0f)*10000+toBCD(p[10])*100+toBCD(p[11])))/100;
	
	imu_cnt++;
	if((imu_cnt%1)==0){
		
        long timestamp_us = get_current_time_us();
        double acc_x = forward_linear_accel * ACCEL_G;
        double acc_y = 0.;
        double vel_z_axis = 0.-z_angular_velocity/180.*PI;
	imu_output_cnt++;
	if(imu_output_cnt>100)
	{
	//printf("2222222222 imu data received====%f\n",z_angle);
	imu_output_cnt = 0;
	}
	z_angle =  z_angle/180.*PI;
		//fprintf(fp_2,"%ld,%s,%.2f,%.4f\n",timestamp_us,"1",z_angle,z_angular_velocity);
////////////////
        //四个参数分别是时间戳，x加速度，y加速度，z轴角速度
        pthread_mutex_lock(&mutex);
		if(p_msg_queue != nullptr) {
            ImuData2D data(timestamp_us, acc_x, acc_y, vel_z_axis);
			data.angle_z = z_angle;
			std::vector<char> tmp;
			data.to_char_array(&tmp);
			p_msg_queue->send(channel, tmp.data(), tmp.size());
        }
        pthread_mutex_unlock(&mutex);
    }
}

void *IMUModule::thread_function(void *param) {

    IMUModule* ptr = (IMUModule *) param;

    //这里是线程参数
    //初始化并启动IMU
    //在这里用 ptr->xxx 的形式访问你定义的中间变量，就像下面while那一行里那样
    //------------------
	int     byte_index=0,len=0,new_frame=0;
	char    in[100];
	char	frame[100];
	char	dev_name[200];
	get_configs()->get_string("imu", "dev_name", dev_name, nullptr);
	ptr->fd=ptr->open_tty("/dev/agv_imu", 115200,0);
	//printf("open_tty(\"/dev/ttyUSB0\",115200,0),fd=%d\n",fd);
	if(ptr->fd==-1){
		printf("IMU failed\n");
		return 0;
	}
	printf("IMU start\n");
    while(ptr->thread_running) {
        //循环读取IMU数据
		int ret=read(ptr->fd,in,100);
		if(ret>0){
			for(int i=0;i<ret;i++){
				if(new_frame==0){
					if(in[i]==0x68){	//收到H
						byte_index=0;
						len=0;
						new_frame=1;
					}
				}else{
					frame[byte_index]=in[i];
					byte_index++;
					if(byte_index==1)len=in[i];
					if(byte_index==len){
						if(ptr->check_CS(frame,len)){
							ptr->handle_frame(frame,len, ptr);
						}else{
							printf("rcv 1 frame.wrong CS\n");
						}
						memset(frame,0,100);
						new_frame=0;
					}
				}
			}
		}
    }
	return nullptr;
};

void IMUModule::close_imu() {
    //在这里关闭imu
    //在这里直接用变量名就可以访问你定义的中间变量
    //如直接调用 thread_running
    //------------------
	close(fd);
};

void IMUModule::set_target(MsgQueue * target, int channel) {
    pthread_mutex_lock(&mutex);
	p_msg_queue = target;
    this->channel = channel;
    pthread_mutex_unlock(&mutex);
}
    
