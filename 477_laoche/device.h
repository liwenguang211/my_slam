#ifndef AGV_DEVICES_H
#define AGV_DEVICES_H

#include <pthread.h>
#include <termios.h>

#include "common/sensor_data.h"
#include "msg_queue.h"
#include "move.h"
#include <list>



class sensor_data{
public:
	char p[100];
	int len;
	int cmd;
	long stamp;
	sensor_data(char *src,int l,int c,long t){
		strncmp(p,src,l);
		len=l;
		cmd=c;
		stamp=t;
	}
};

class DevicesModule : public MsgQueueListener
{

public:
    DevicesModule(MsgQueue *queue);
    ~DevicesModule();

    //MOVE Message
    virtual void recieve_message(const int channel, char *buf, const int size);

private:
    MsgQueue *p_msg_queue;

    int odom_imu_fd = -1;
    int move_fd = -1;
    int nav_fd = -1;

    pthread_t odom_imu_thread;
    pthread_t nav_thread;
    bool thread_running = true;

    static void *thread_function_odom(void *param);
    static void *thread_function_nav(void *param);
	int  toBCD(int x);
    long handle_frame_odom_imu(char *p, int len, int msqid);
    long handle_frame_odom3(char *p, int len);
    void send_wheel_data(WheelSensorData d);

    pthread_mutex_t mutex;
    int handle_movement_message(int speed, int turn_left);
    int handle_turn_message(int dir,double angle);
    int handle_collision_message(int level);
    int handle_lift_message(int dir);
	int handle_lift_height_message(int height);
	int handle_dio2_message(int angle);
	int handle_nav_message(double startx,double starty,double starttheta,double endx,double endy,double endtheta,int dir);
	void queue_msg(sensor_data d);
	void handle_sensor_data(sensor_data d);
	std::list<sensor_data> dts;
	int should_send=0;

	
    long last_left_data = 0;
    long last_left_timestamp = 0;
    long last_right_data = 0;
    long last_right_timestamp = 0;
    long odometer_input_range = 50000;
    long l_send_time = 0, r_send_time = 0, time_ms = 0;
	int cnt_imu;
    double accel_g = 9.80665;
	double last_imu_angle;
    speed_t getBaudrate(int baudrate);
    int open_tty(char const *path, int baudrate, int flags);
};

#endif
