#ifndef NAV_MODULE_H
#define NAV_MODULE_H

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <list>

#include <Eigen/Core>

#include "common/pose.h"
#include "common/capture_point.h"
#include "msg_queue.h"


class NavModuleOption {
public:
    MsgQueue *      msg_queue;
    int             pose_channel;
    int             arrive_channel;
	CapRoute *		route;
	RouteChain		chain;
};

class NavModule : public MsgQueueListener {
public:
    NavModule(NavModuleOption &opt);
    ~NavModule();

    int handle_radar_data(PointCloudData & data);
    int handle_imu_data(ImuData2D & data);
    virtual void recieve_message(const int channel, char *buf, const int size);
	
    MsgQueue *      msg_queue;
	
private:
	NavModuleOption option;
    pthread_mutex_t sensor_mutex;
	
	static void * update_pose_thread_function(void * param);
	pthread_t nav_update_pose_thread;
	pthread_t handle_operation_thread;
    int pose_listener_id;
    int arrive_listener_id;
	
///////////////////////////////////////////////////////////////////////////////
	
    int sleep_count = 0;

    int emergency_level = 0; //1：前进速度减半；2：停止前进、角速度减半；3：完全停止
    int stop_count = 0;

    void send_current_route();
    void send_movement(double v, double w);
	void send_nav_msg(double start_x, double start_y,double start_theta,double end_x,double end_y,int dir);
	void send_turn_msg(int dir, double angle);
	void send_lift_msg(int dir);

	///////clm0703
	static int count;
	void cap_path_planning2(CapRoute& cr);
	void cap_path_planning_new_if(CapRoute& cr);
	static void *handle_operation_thread_function(void *param);
	std::list<Action_clm> 	actions;//这里保存要下发给新彪的点
	std::list<CapPoint> caps;
	int got_first_pos=0;
	
	RouteChain	chain;
	CapRoute caproute;//这里保存传下来的路径
	
	bool operation_finished=false;
	bool lift_up_finished=false;
	bool lift_down_finished=false;
	
};

#endif
