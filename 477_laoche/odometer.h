#ifndef AGV_ODOMETER_MODULE_H
#define AGV_ODOMETER_MODULE_H

#include <list>

#include "common/pose.h"
#include "common/sensor_data.h"
#include "msg_queue.h"

class Odometer : public MsgQueueListener {
public:
    Odometer(MsgQueue *queue, int imu_chan, int wheel_chan, int odom_chan);
    ~Odometer();

    int handle_wheel_data(WheelSensorData & data);
    int handle_imu_data(ImuData2D & data);
    
    virtual void recieve_message(const int channel, char *buf, const int size);
	bool start_mapping;
	double obtain_imu_angle();
private:
    std::list<WheelSensorData> left_datas;
    std::list<WheelSensorData> right_datas;
    std::list<ImuData2D> imu_datas;
    pthread_mutex_t list_mutex;

    int skip = 0;
    int count = 0;

    MsgQueue *queue;
    int imu_c;
    int wheel_c;
    int odom_c;
    float encoder_per_round;
    float diameter;
           bool first_read;
	float distance;
	long last_left, last_right, last_t;
	long cur_t, cur_left, cur_right;
	bool left_ready,right_ready,imu_ready;
	double cur_imua,last_imua;
	double start_mapping_angle;
	
    long linear_interpolation(std::list<WheelSensorData> *datas, long time);

    int do_DR(Position *buf, long delta_t, long delta_left, long delta_right, double delta_theta);
};


#endif

