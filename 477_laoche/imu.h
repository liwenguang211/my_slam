#ifndef AGV_IMU_H
#define AGV_IMU_H

#include <termios.h>

#include <pthread.h>
#include <map>

#include "common/sensor_data.h"
#include "msg_queue.h"

class IMUModule {
public:
    IMUModule();
    ~IMUModule();

    void set_target(MsgQueue * target, int channel);
    
private:
    static void *thread_function(void *param);
    void close_imu();
    pthread_t imu_thread;
    bool thread_running = true;

    //把其他的中间变量定义在这里
    //比如文件句柄，计数之类的
    //-----------------
	int fd;
	char temp[100];
    float z_angular_velocity;
    float forward_linear_accel;
    float z_angle;
    int imu_cnt=0;
	
    speed_t getBaudrate(int baudrate);
    void handle_frame(char* p,int len,IMUModule* ptr);
	int check_CS(char* p,int len);
    int open_tty(char const *path, int baudrate, int flags);
    int toBCD(int x);
	int imu_output_cnt;
    MsgQueue * p_msg_queue = nullptr;
    int channel;
    pthread_mutex_t mutex;
};

#endif

