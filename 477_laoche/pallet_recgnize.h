#ifndef PALLET_RECGNIZE_H
#define PALLET_RECGNIZE_H
#include <assert.h>
#include <vector>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <termios.h>
#include "common/simple_grid_map.h"
#include "radar.h"
#include "common/pose.h"
#include "common/time.h"
using namespace std;


class Pallet_recgnize : public MsgQueueListener {
public:
	//Amcl_gl() ;
	Pallet_recgnize(MsgQueue *queue, int radar_chan);
    virtual void recieve_message(const int channel, char *buf, const int size);
	~Pallet_recgnize();
public:
	SimpleGridMap *gridmap;
	bool para_initial();
	PointCloudData last_radar_data;
	int laser_recv_cnt = 10;
	float ini_ang = 60;
	float end_ang = 480;
	float ang_step = 0.5;
	float pallet_width = 0.2;
	float pallet_lenth =0.23;
	int radar_data_dispose();
	float cloudCurvature[540];
	int cloudSortInd[540];
	int cloudNeighborPicked[540];
	int cloudLabel[540];

	Position mean_foot1;
	Position mean_foot2;
	bool recognized_start;
	bool pose_recognized;
	float cal_disp(double x1,double y1);
	int recognize_cnt;
	int max_recog_cnt;
	int pallet_recognize(float len,float width,float angle_sech,int clc_cnt,Position & result_pose);////
	int legs_recognize(float len,float width,float angle_sech,int clc_cnt,Position & leg1_pose,Position & leg2_pose);////
	float search_angle;
private:
	MsgQueue *queue_a;
	bool lasers_update_;
	int radar_c;
	pthread_mutex_t listen_pose_mutex;
	Position radar_position;
};
/////////////////////////////////////////////////////////////////////////////////


#endif
