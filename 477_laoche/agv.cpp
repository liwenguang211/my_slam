#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <random>
#include <fstream>
#include <iostream>
#include <ceres/cubic_interpolation.h>

#include "agv.h"
#include "common/configuration.h"
#include "scan_matching.h"
#include "common/distance_map.h"
FILE *fp_scanmatching_pos;
FILE *fp_dr_pos;

//#define TEST_MODE


void AGVInstance::initialize()
{
    if (state != SYSTEM_STATE_OFFLINE)
    {
        return;
    }
	fp_scanmatching_pos = fopen("scan_matching_pos.txt","w");
	fp_dr_pos = fopen("dr_pos.txt","w");

    //initialize mutex
    pthread_mutex_init(&mutex_state, nullptr);
    pthread_mutex_init(&mutex_pose, nullptr);
    pthread_mutex_init(&mutex_map, nullptr);
    pthread_mutex_init(&mutex_radar_data, nullptr);
    pthread_mutex_init(&mutex_odom_list, nullptr);
    pthread_mutex_init(&mutex_raw_odom, nullptr);

    //initialze message queue
    message_queue = new MsgQueue(16);

#ifndef TEST_MODE
    //initialize modules
    devs = new DevicesModule(message_queue);

    radar = new RadarModule();
    radar->set_target(message_queue, CHANNEL_RADAR);

    odom = new Odometer(message_queue, CHANNEL_IMU, CHANNEL_WHEEL, CHANNEL_ODOM);
    amcl_global_locate = new Amcl_gl(message_queue,CHANNEL_RADAR,CHANNEL_ODOM);
	my_runcurve = new curve_run(message_queue,CHANNEL_MOVE);
	
	pallet_pose_recgnize  =  new Pallet_recgnize(message_queue,CHANNEL_RADAR);
	usleep(20000);
	odom->start_mapping = true;
	/*Position poseraw;
	poseraw.timestamp = get_current_time_us();
	poseraw.x = 0;
	poseraw.y = 0;
	poseraw.theta = 0;
	set_raw_odom_position(poseraw);
	*/
    //set flag
    state = SYSTEM_STATE_FREE;

    //initialize listener
    registe_listener();
    // message_queue->print_debug_string();
	
	imu = new IMUModule();
    imu->set_target(message_queue, CHANNEL_IMU);
#endif
	st=new storage();
	double radar_x = get_configs()->get_float("radar", "radar_position_x", nullptr);
    double radar_y = get_configs()->get_float("radar", "radar_position_y", nullptr);
    double radar_theta = get_configs()->get_float("radar", "radar_position_theta", nullptr);
	
	radar_position.x = radar_x;
	radar_position.y = radar_y;
	radar_position.theta = radar_theta;
	insert_index = 0;
	on_scan_matching_cnt_  =-10;
	scan_init = false;
	record_pose_cnt = 0;
	loadPoseFromserver();
	scan_matching_cnt = 0;
	motor = new Motor(message_queue,  CHANNEL_MOTOR_CTL,CHANNEL_WHEEL,CHANNEL_MOVE);
//	pthread_create(&pallet_recognize_thread, NULL, pallet_recognize_thread_function, this);

}

#define DEL(p)        \
    if (p != nullptr) \
    {                 \
        delete p;     \
        p = nullptr;  \
    }

void AGVInstance::shutdown()
{
    if (state == SYSTEM_STATE_OFFLINE)
    {
        return;
    }
    pthread_mutex_lock(&mutex_state);

    //stop threads
    if (painting_map_thread_running)
    {
        painting_map_thread_running = false;
        pthread_join(painting_map_thread, NULL);
    }

    //free modules
    DEL(carto);
    DEL(odom);
    DEL(devs);
    DEL(radar);

    //free message queue
    message_queue->clear();
    delete message_queue;

    //delete mutex
    pthread_mutex_unlock(&mutex_state);
    pthread_mutex_destroy(&mutex_state);
    pthread_mutex_destroy(&mutex_pose);
    pthread_mutex_destroy(&mutex_map);
    pthread_mutex_destroy(&mutex_radar_data);
    pthread_mutex_destroy(&mutex_odom_list);
    pthread_mutex_destroy(&mutex_raw_odom);

    //set flag
    state = SYSTEM_STATE_OFFLINE;
}

int AGVInstance::get_system_state()
{
    pthread_mutex_lock(&mutex_state);
    int ret = state;
    pthread_mutex_unlock(&mutex_state);
    return ret;
}

//--------------------------------------------------------------------

void AGVInstance::registe_listener()
{
    message_queue->add_listener(CHANNEL_POSE, this);
    message_queue->add_listener(CHANNEL_STATE, this);
    message_queue->add_listener(CHANNEL_RADAR, this);
    message_queue->add_listener(CHANNEL_ODOM, this);
    // message_queue->print_debug_string();
}

void AGVInstance::recieve_message(const int channel, char *buf, const int size)//例行的,不受状态变化影响
{
    if (channel == CHANNEL_STATE)
    {
        Position pose;
        pose.from_char_array(buf, size);
        update_last_position(pose);
		
		
		int arg_int1=*((int*)buf);
		int arg_int2=*((int*)(buf+4));
		int arg_int3=*((int*)(buf+8));
		int arg_int4=*((int*)(buf+12));
		int arg_int5=*((int*)(buf+16));
//		printf("###mcu state.输入=%d,输出=%d,电量=%d,速度=%d,角速度=%d\n",arg_int1,arg_int2,arg_int3,arg_int4,arg_int5);
		mcu_input=arg_int1;
		mcu_output=arg_int2;
		mcu_battery_level=arg_int3;
		mcu_speed=arg_int4;
		mcu_omega=arg_int5;
    }
    if (channel == CHANNEL_POSE)
    {
        Position pose;
        pose.from_char_array(buf, size);
	if (get_system_state() == SYSTEM_STATE_MAPPING||(get_system_state() == SYSTEM_STATE_INIT_POSE))//////如果是建图模式，则使用建图过程中的scan-match位姿修正里程计。否则，里程计修正按照10次校正一次
	//if (get_system_state() == SYSTEM_STATE_INIT_POSE)//////如果是建图模式，则使用建图过程中的scan-match位姿修正里程计。否则，里程计修正按照10次校正一次
        if (!get_current_map().is_nullptr())
	update_last_position(pose);
	//else
	//flush_odom_data();
        // printf("recieve pose %ld (%.3f, %.3f, %.3f)\n", pose.timestamp, pose.x, pose.y, pose.theta);
    }
    else if (channel == CHANNEL_ODOM)
    {
        Position pose;
        pose.from_char_array(buf, size);
        

		double delta_tt = (pose.delt_t)*0.000001;
		//if((pose.timestamp-last_time_pose)<0) return;
        add_odom_data(pose);
		add_raw_odom_data(pose);
		if(pose.delt_t<=0)
		{if(robot_stop_cnt<65535)robot_stop_cnt++;home_tuning = false;}
		//else if(vth>fuzzy_mini && (vx<fuzzy_mini)&& (vy<fuzzy_mini))
		//{robot_stop_cnt = 0;home_tuning = true;insert_index = 0;}
		else
		{robot_stop_cnt = 0;home_tuning = false;}
		last_time_pose = pose.timestamp;
		//if((vx>fuzzy_mini)|| (vy>fuzzy_mini)) 
		//printf("线速度vx=%f,vy=%f,delta_t=%f\n",vx,vy,pose.theta*1000000);
		record_pose_cnt ++;
		if(record_pose_cnt>100)
		{
			record_pose_cnt = 0;
			savePoseToServer();
		}
	//if (get_current_map().is_nullptr())////防止scan-matching没有启动，导致odom_list不断插入数据，占用cpu过高
	if(odom_list.size()>200)
	 {
	//printf("清空里程计缓冲区0000000pos2d,x=%f,y=%f,th=%f\n",pose2d.x,pose2d.y,pose2d.theta);
	flush_odom_data();
	//printf("清空里程计缓冲区pos2d,x=%f,y=%f,th=%f\n",pose2d.x,pose2d.y,pose2d.theta);
	}
    }
    else if (channel == CHANNEL_RADAR)
    {
///////////////////////首先判断机器人的运行距离，这里先写死距离方向0.01m触发和角度方向0.1弧度触发
		Position pose_scan;
		Position pose_amcl_;
		Position pose_scan_correct;
		bool pos_correct=false;
		cur_pos = expolate_current_position();

		//if (get_system_state() == SYSTEM_STATE_MAPPING)
		//fprintf(fp_dr_pos,"%ld	%f	%f	%f	%f	%f	%f\n",cur_pos.timestamp,cur_pos.x,cur_pos.y,cur_pos.theta,raw_odom.x,raw_odom.y,raw_odom.theta);

		//printf("11111SYSTEM_STATE_INIT_POSE=%d on_scan_matching_cnt_=%d\n",get_system_state(),on_scan_matching_cnt_);

		if(get_system_state() == SYSTEM_STATE_INIT_POSE) return;
		//printf("SYSTEM_STATE_INIT_POSE=%d on_scan_matching_cnt_=%d\n",get_system_state(),on_scan_matching_cnt_);
		if(on_scan_matching_cnt_ >= 50)//////机器人初始化完成后，启动scan-matching计数
		scan_matching_cnt++;
		//if(scan_matching_cnt>20) flush_odom_data();//////避免机器人长时间未动，导致odomlist不断增多，占用cpu过高
		//if ((fabs(cur_pos.x - last_pos.x) > 0.01) || (fabs(cur_pos.y - last_pos.y) > 0.01) || (fabs(cur_pos.theta - last_pos.theta) > 0.1) || (!scan_init) || (on_scan_matching_cnt_<50)||(robot_stop_cnt<10))
		if ((!scan_init) || (on_scan_matching_cnt_<50)||(robot_stop_cnt<70))

		{
			
			PointCloudData data(0);
			data.from_char_array(buf, size);
			pthread_mutex_lock(&mutex_radar_data);
			for (auto & point : data.points) {
				point(0) += radar_position.x;
			}
			last_radar_data = data;
			pthread_mutex_unlock(&mutex_radar_data);
			if (get_system_state() != SYSTEM_STATE_MAPPING)
			{
					do_scan_matching(data, pose_scan,pos_correct);
					scan_matching_cnt = 0;
			}
            if(get_system_state() == SYSTEM_STATE_MAPPING_LANDMARK)
				handle_landmark_data_on_painting(data);
			
			last_pos = expolate_current_position();
			if(pos_correct && on_scan_matching_cnt_<50)
			{
			on_scan_matching_cnt_++;
			update_last_position(pose_scan);
			printf("执行on_scan_matching_cnt_++\n");
			}
			
			if (on_scan_matching_cnt_ >= 50 && pos_correct)
			{/////开机scan-matching结束后，再启动位姿矫正
				if(!scan_init)
				{
					//update_last_position(pose_scan);
					printf("初始定位位姿为%f--%f--%f\n",pose_scan.x,pose_scan.y,pose_scan.theta);
					scan_init = true;
					//state = SYSTEM_STATE_FREE;
					return;
				}
				on_scan_matching_cnt_ =100;
				raw_pose.odom_raw_pos[insert_index] = expolate_current_position_tim(data.timestamp);
				raw_pose.scan_match_raw_pos[insert_index] = pose_scan;
				pose_amcl_ = amcl_global_locate->expolate_current_position_tim(data.timestamp);
				//pose_amcl_ = amcl_global_locate->locate_pose;
				fprintf(fp_scanmatching_pos,"%ld	%d	%f	%f	%f	%f	%f	%f	%f	%f	%f\n",data.timestamp,1,pose_scan.x,pose_scan.y,pose_scan.theta,pose_amcl_.x,pose_amcl_.y,pose_amcl_.theta,raw_odom.x,raw_odom.y,raw_odom.theta);
				
				//if(fabs(pose_amcl_.x-pose_scan.x)>0.15&&fabs(pose_amcl_.y-pose_scan.y)>0.15&&fabs(pose_amcl_.theta-pose_scan.theta)>0.15)
				{
				//pose_scan = pose_amcl_;///amcl_global_locate->locate_pose;//expolate_current_position_tim(data.timestamp);
				//printf("tolerance error is too large pose_scan is amcl pos!!!!!!!!!!!!!!!\n");
				}
				//if(fabs(pose_amcl_.theta-pose_scan.theta)>0.3)
				//pose_scan.theta = pose_amcl_.theta ;
				//pose_scan = pose_amcl_;///amcl_global_locate->locate_pose;//expolate_current_position_tim(data.timestamp);
				//fprintf(fp_scanmatching_pos,"%f,%f,%f\n",raw_pose.odom_raw_pos[insert_index].x,raw_pose.odom_raw_pos[insert_index].y,raw_pose.odom_raw_pos[insert_index].theta);
				//fprintf(fp_dr_pos,"%f,%f,%f\n",pose_scan.x,pose_scan.y,pose_scan.theta);
				
				insert_index++;
				//if(home_tuning == true)
				//{
				insert_index = 0;
				update_last_position(pose_scan);
				//}
				//printf("机器人执行校正+++++\n");
				if (insert_index >= 10)////////////////执行位姿数据融合处理
				{
					pose_scan_correct = raw_pos_correct(raw_pose,10);//////计算修正的数据
					pose_scan_correct.timestamp = data.timestamp;
					//printf("校正前的坐标为x%f--y%f--theta%f\n", pose_scan.x, pose_scan.y, pose_scan.theta);
					//printf("pose_scan_correct校正的坐标为x%f--y%f--theta%f\n", pose_scan_correct.x, pose_scan_correct.y, pose_scan_correct.theta);
					update_last_position(pose_scan_correct);
					//update_last_position(pose_scan);
					last_pos = expolate_current_position();////修正位姿数据的同时，将修正后的位姿赋给上一步的位姿
					insert_index = 0;
				}

			}
		}
    }
}

//--------------------------------------------------------------------

void AGVInstance::load_grid_map(SimpleGridMap &map)
{
    MapSharedPointer pointer(new MapSharedMemory(map));
    pthread_mutex_lock(&mutex_map);
    p_map = pointer;
    pthread_mutex_unlock(&mutex_map);
}

void AGVInstance::load_grid_map(char *buf, int size)
{
    MapSharedPointer pointer(new MapSharedMemory(buf, size));
    pthread_mutex_lock(&mutex_map);
    p_map = pointer;
    pthread_mutex_unlock(&mutex_map);
}

bool AGVInstance::init_locate()
{
	insert_index = 0;
	on_scan_matching_cnt_  =-10;
	scan_init = false;
	record_pose_cnt = 0;
	//loadPoseFromserver();
	scan_matching_cnt = 0;

}
//--------------------------------------------------------------------
Position AGVInstance::expolate_current_position_tim(long timestamp)
{
	pthread_mutex_lock(&mutex_pose);
	Position ret = pose2d;

	pthread_mutex_lock(&mutex_odom_list);
	for (Position &p : odom_list)
	{
		if (p.timestamp > ret.timestamp && (p.timestamp <= timestamp))
		{
			ret = ret * p;
		}

	}
	pthread_mutex_unlock(&mutex_odom_list);
	pthread_mutex_unlock(&mutex_pose);
	return ret;
}
Position AGVInstance::expolate_current_position()
{
    // Position tmp;
    // long timestamp = get_current_time_us();
    // int ret = odom->get_odometer()->expolate_pose(timestamp, &tmp);
    // if(ret != 0) {
    //ERROR
    // printf("Expolate current pose failed. Use last position\n");
    // return get_last_position();
    // }
    // return tmp;
    pthread_mutex_lock(&mutex_pose);
    Position ret = pose2d;
    pthread_mutex_lock(&mutex_odom_list);
    for (Position &p : odom_list)
    {
        if (p.timestamp > ret.timestamp)
        {
            ret = ret * p;
        }
    }
    pthread_mutex_unlock(&mutex_odom_list);
    pthread_mutex_unlock(&mutex_pose);
    return ret;
}
//
Position AGVInstance::get_last_position()
{
    pthread_mutex_lock(&mutex_pose);
    Position ret = pose2d;
    pthread_mutex_unlock(&mutex_pose);
    return ret;
}

void AGVInstance::update_last_position(Position &pose)
{
    update_last_position(pose, 1, 1);
}
//最新的scanmatching结果,当前位姿要将后面的位姿变化乘上去
void AGVInstance::update_last_position(Position &pose, int my_level, int change_level_to)
{
    pthread_mutex_lock(&mutex_pose);
    // if(pose.timestamp > pose2d.timestamp) {
    if (pose_level <= my_level)
    {
        if (pose.timestamp > pose2d.timestamp)
        {
            pose2d = pose;
            update_odom_list_timestamp(pose.timestamp);
        }
        pose_level = change_level_to;
    }
    // }
    pthread_mutex_unlock(&mutex_pose);
}

void AGVInstance::add_odom_data(Position &pose_change)
{
    long timestamp = pose_change.timestamp;
    pthread_mutex_lock(&mutex_pose);
    /*if (timestamp < pose2d.timestamp)
    {
	
        pthread_mutex_unlock(&mutex_pose);
        return;
    }*/
    pthread_mutex_lock(&mutex_odom_list);
    if (odom_list.empty())
    {
        odom_list.push_back(pose_change);
    }
    else
    {
        for (auto iter = odom_list.begin();; iter++)
        {
            if (iter == odom_list.end() || iter->timestamp >= timestamp)
            {
                odom_list.insert(iter, pose_change);
                break;
            }
        }
    }
    pthread_mutex_unlock(&mutex_odom_list);
    pthread_mutex_unlock(&mutex_pose);
}

void AGVInstance::update_odom_list_timestamp(long timestamp_us)
{
    pthread_mutex_lock(&mutex_odom_list);
    while (!odom_list.empty())
    {
        if (odom_list.front().timestamp > timestamp_us)
        {
            break;
        }
        odom_list.pop_front();
    }
    pthread_mutex_unlock(&mutex_odom_list);
}

void AGVInstance::flush_odom_data()
{
    pthread_mutex_lock(&mutex_pose);
    Position pose = pose2d;
    pthread_mutex_lock(&mutex_odom_list);
    if (pose_level <= 1)
    {
        while (!odom_list.empty())
        {
            pose = pose * odom_list.front();
            odom_list.pop_front();
        }
        pose2d = pose;
    }
    else
    {
        odom_list.clear();
    }
    pthread_mutex_unlock(&mutex_odom_list);
    pthread_mutex_unlock(&mutex_pose);
}

void AGVInstance::add_raw_odom_data(Position &pose_change) {
    pthread_mutex_lock(&mutex_raw_odom);
   // if(pose_change.timestamp > raw_odom.timestamp) {
        raw_odom = raw_odom * pose_change;
	raw_odom.theta = obtain_imu_angle();
    //}
	//printf("cur raw_odom000000000000000000000000000000==x=%f===y=%f===theta=%f====delta=%f\n",raw_odom.x,raw_odom.y,raw_odom.theta,pose_change.theta);
    pthread_mutex_unlock(&mutex_raw_odom);
}

Position AGVInstance::get_raw_odom_position() {
    pthread_mutex_lock(&mutex_raw_odom);
    Position ret = raw_odom;
    pthread_mutex_unlock(&mutex_raw_odom);
    return ret;
}

bool AGVInstance::set_raw_odom_position(Position &pose) {
    pthread_mutex_lock(&mutex_raw_odom);
    //if(pose.timestamp > raw_odom.timestamp) {
        raw_odom = pose;
        pthread_mutex_unlock(&mutex_raw_odom);
        return true;
    //}
    pthread_mutex_unlock(&mutex_raw_odom);
    return false;
}

//--------------------------------------------------------------------

PointCloudData AGVInstance::get_last_radar_data()
{
    PointCloudData ret;
    pthread_mutex_lock(&mutex_radar_data);
    ret = last_radar_data;
    pthread_mutex_unlock(&mutex_radar_data);
    return ret;
}

//--------------------------------------------------------------------
bool AGVInstance::start_navigating(CapRoute *r)//todo 测试通过后删除此入口
{
	pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_FREE)
    {
        state = SYSTEM_STATE_NAVIGATING;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);
	
	
	NavModuleOption opt;
    opt.msg_queue = message_queue;
    opt.pose_channel = CHANNEL_POSE;
    opt.arrive_channel = CHANNEL_ARRIVE;
	opt.route=r;
	nav=new NavModule(opt);
}
bool AGVInstance::start_navigating(RouteChain c)
{
	pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_FREE)
    {
        state = SYSTEM_STATE_NAVIGATING;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);
	
	NavModuleOption opt;
    opt.msg_queue = message_queue;
    opt.pose_channel = CHANNEL_POSE;
    opt.arrive_channel = CHANNEL_ARRIVE;
	opt.chain=c;
	nav=new NavModule(opt);
}

bool AGVInstance::stop_navigating()
{
    pthread_mutex_lock(&mutex_state);
	nav_thread_running = false;
    if (state == SYSTEM_STATE_NAVIGATING)
    {
		state=SYSTEM_STATE_FREE;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);
	DEL(nav);
	exec_route=0xffff;
	exec_cap=0;
    return true;
}
double AGVInstance::obtain_imu_angle()
{
	double ret_ang = odom->obtain_imu_angle();
	return ret_ang;
}

bool AGVInstance::start_mapping()
{
    pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_FREE)
    {
        state = SYSTEM_STATE_MAPPING;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);
	amcl_global_locate->_begin_map_create();
    DEL(carto);
    char carto_dir[200];
    get_configs()->get_string("carto", "config_files_dir", carto_dir, nullptr);
    char carto_name[200];
    get_configs()->get_string("carto", "config_file_name", carto_name, nullptr);
    CartoModuleOption opt(carto_dir, carto_name);
    opt.radar_scan_time = get_configs()->get_float("carto", "radar_scan_time", nullptr);
    opt.pose_mathching_score = get_configs()->get_float("carto", "pose_matching_score", nullptr);
    opt.msg_queue = message_queue;
    opt.pose_channel = CHANNEL_POSE;
    opt.imu_channel = CHANNEL_IMU;
    opt.radar_channel = CHANNEL_RADAR;
    carto = new CartoModule(opt);

    landmark_radar_datas.clear();

	flush_odom_data();
	pose2d.x = 0;
	pose2d.y = 0;
	pose2d.theta = 0;
	

	//if(my_karto!=NULL) DEL(my_karto);
	//my_karto = new SlamKarto();
	Position poseraw;
	poseraw.timestamp = get_current_time_us();
	poseraw.x = 0;
	poseraw.y = 0;
	poseraw.theta = 0;
	set_raw_odom_position(poseraw);
	
	odom->start_mapping = true;
    	// set_raw_odom_position(Position(get_current_time_us(), 0., 0., 0.));

    painting_map_thread_running = true;
    pthread_create(&painting_map_thread, nullptr, painting_map_thread_function, this);

    return true;
};
bool AGVInstance::stop_mapping()
{
    pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_MAPPING)
    {
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);
	amcl_global_locate->_end_map_create();
    painting_map_thread_running = false;
    return true;
}
bool AGVInstance::start_mapping_landmark()
{
	if (get_current_map().is_nullptr()) return false;

    pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_FREE&&pose_initilized)
    {
        state = SYSTEM_STATE_MAPPING_LANDMARK;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);

    return true;
};
bool AGVInstance::stop_mapping_landmark()
{
    long start_time = get_current_time_us();
	long opt_time;
    MapSharedPointer p_map;
    p_map = get_current_map();
    LandmarkMapGenerator gen(get_configs()->get_float("map", "landmark_map_resolution", nullptr));
    int frame_count = 0;
    int point_count = 0;
    for (LandmarkRadarData &data : landmark_radar_datas)
    {
        Position pose(1, data.pose[0], data.pose[1], data.pose[2]);
        PointCloudData radardata(1);
        for (double d : data.points)
        {
            double farray = d;
            float *p_float = (float *)(&farray);
            radardata.add_point(p_float[0], p_float[1]);
        }
        ScanMatchingOptions opt;
        opt.occupied_space_cost_factor = get_configs()->get_float("scan_matching", "occupied_space_cost_factor", nullptr);
        opt.translation_delta_cost_factor = get_configs()->get_float("scan_matching", "translation_delta_cost_factor", nullptr);
        opt.rotation_delta_cost_factor = get_configs()->get_float("scan_matching", "rotation_delta_cost_factor", nullptr);
        opt.num_threads = get_configs()->get_int("scan_matching", "num_threads", nullptr);
        opt.max_num_iterations = get_configs()->get_int("scan_matching", "max_num_iterations", nullptr);

        Position optimism = scan_matching(pose, p_map.get(), radardata, opt);
        for (auto &point : radardata.points)
        {
            Position p(1, point(0), point(1), 0);
            Position after_trans = optimism * p;
            gen.add_point(after_trans.x, after_trans.y);
            point_count++;
		//fprintf(fp_t,"%ld,%ld\n",after_trans.x, after_trans.y);
        }
        frame_count++;
    }
    p_map->landmark = gen.to_landmark_map();
    printf("Optimism done, draw %d landmark frames\n", frame_count);
    opt_time = (get_current_time_us() - start_time) / 1000;
    printf("Using time %lds,%ldms\n", opt_time / 1000, opt_time % 1000);
    usleep(500000);
    state = SYSTEM_STATE_FREE;
    return true;
};
void AGVInstance::handle_landmark_data_on_painting(PointCloudData &data)
{
    float intensity_threshold = get_configs()->get_float("radar", "intensity_threshold", nullptr);
    float landmark_point_rate = get_configs()->get_float("landmark_scan_matching", "point_rate", nullptr);
    LandmarkRadarData tmp;
    Position pose = expolate_current_position();
    tmp.pose[0] = pose.x;
    tmp.pose[1] = pose.y;
    tmp.pose[2] = pose.theta;
    int size = data.points.size();
    for (int i = 0; i < size; i++)
    {
        if (data.intensities[i] > intensity_threshold)
        {
            double point;
            float *p_float = (float *)(&point);
            p_float[0] = data.points[i](0);
            p_float[1] = data.points[i](1);
            tmp.points.push_back(point);
        }
    }
    if (tmp.points.size() > (int)(size * landmark_point_rate))
    {
        landmark_radar_datas.push_back(tmp);
    }
}

void *AGVInstance::painting_map_thread_function(void *param)//定时从cartographer提取地图,反光板优化
{
	//FILE *fp_t;
	//fp_t = fopen("refletc_points.txt","w");
    AGVInstance *ptr = (AGVInstance *)param;
    long time_us = get_configs()->get_long("carto", "paint_map_period_us", nullptr);
	long start_time = get_current_time_us();
	long opt_time;
    usleep(time_us);
	MapSharedPointer p_map;
	usleep(1000000);
    while (ptr->painting_map_thread_running)
    {
        std::vector<char> map_data;
        ptr->carto->paint_map(&map_data);
        ptr->load_grid_map(map_data.data(), map_data.size());
        usleep(time_us);
    }
    ptr->state = SYSTEM_STATE_OPTIMISM;
    start_time = get_current_time_us();
    ptr->carto->stop_and_optimize();
    opt_time = (get_current_time_us() - start_time) / 1000;
    printf("Using time %lds,%ldms\n", opt_time / 1000, opt_time % 1000);
    if (ptr->carto)
    {
        std::vector<char> map_data;
        ptr->carto->paint_map(&map_data);
        ptr->load_grid_map(map_data.data(), map_data.size());
    }
    DEL(ptr->carto);
    printf("Optimism landmarks\n");
    start_time = get_current_time_us();
    
	
    
    p_map = ptr->get_current_map();
    /*
    LandmarkMapGenerator gen(get_configs()->get_float("map", "landmark_map_resolution", nullptr));
    int frame_count = 0;
    int point_count = 0;
    for (LandmarkRadarData &data : ptr->landmark_radar_datas)
    {
        Position pose(1, data.pose[0], data.pose[1], data.pose[2]);
        PointCloudData radardata(1);
        for (double d : data.points)
        {
            double farray = d;
            float *p_float = (float *)(&farray);
            radardata.add_point(p_float[0], p_float[1]);
        }
        ScanMatchingOptions opt;
        opt.occupied_space_cost_factor = get_configs()->get_float("scan_matching", "occupied_space_cost_factor", nullptr);
        opt.translation_delta_cost_factor = get_configs()->get_float("scan_matching", "translation_delta_cost_factor", nullptr);
        opt.rotation_delta_cost_factor = get_configs()->get_float("scan_matching", "rotation_delta_cost_factor", nullptr);
        opt.num_threads = get_configs()->get_int("scan_matching", "num_threads", nullptr);
        opt.max_num_iterations = get_configs()->get_int("scan_matching", "max_num_iterations", nullptr);

        Position optimism = scan_matching(pose, p_map.get(), radardata, opt);
        for (auto &point : radardata.points)
        {
            Position p(1, point(0), point(1), 0);
            Position after_trans = optimism * p;
            gen.add_point(after_trans.x, after_trans.y);
            point_count++;
		//fprintf(fp_t,"%ld,%ld\n",after_trans.x, after_trans.y);
        }
        frame_count++;
    }
    p_map->landmark = gen.to_landmark_map();
    printf("Optimism done, draw %d landmark frames\n", frame_count);
    opt_time = (get_current_time_us() - start_time) / 1000;
    printf("Using time %lds,%ldms\n", opt_time / 1000, opt_time % 1000);
    */
	//clm,保存地图至本地,从停止建图移至此处
	printf("###id=%d,name=%s\n",get_global_agv_instance()->map_id,get_global_agv_instance()->map_name);
	if((get_global_agv_instance()->map_id<=40)&&(get_global_agv_instance()->map_id>=1)&&(get_global_agv_instance()->map_name[0]!=0)){
		std::vector<char> map_data;
		//保存地图
//		get_global_agv_instance()->latest_map->get_data(&map_data);
		p_map->to_char_array(&map_data);
		int size = map_data.size();
		printf("###size=%d\n",size);
		char *p = map_data.data();
		FILE* fp;
		if((fp=fopen(get_global_agv_instance()->map_name,"wb"))==NULL){
			printf("error open file\n");
		}else{
			
			int ret;
			int write_len=0;
			while(write_len<size){
				ret=fwrite(p+write_len,1,size-write_len,fp);
				write_len+=ret;
			}
			fclose(fp);
			get_global_storage()->modify_map_slot(get_global_agv_instance()->map_id,get_global_agv_instance()->map_name);
			memset(get_global_agv_instance()->map_name,0,1);
			get_global_agv_instance()->map_id=0;
		}
	}
	

    ptr->state = SYSTEM_STATE_FREE;
	ptr->init_locate();
    return nullptr;
	
}


void AGVInstance::start_global_locating()
{
    pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_FREE)
    {
        state = SYSTEM_STATE_INIT_POSE;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return;
    }
    pthread_mutex_unlock(&mutex_state);
    MapSharedPointer p_map;
    p_map = get_current_map();
    pose_initilized = false;
	//Low_resolution my_lowreso(p_map.get(),0.6);
	amcl_global_locate->gridmap = p_map.get();
	//amcl_global_locate->gridmap = my_lowreso.low_reso_map();

/*	std::vector<char> map_data;
	map_data.clear();
    	 amcl_global_locate->gridmap->to_char_array(&map_data);
	std::ofstream ofile("00locating_map");
    	ofile.write(map_data.data(), map_data.size());
   	 ofile.close();
*/
	amcl_global_locate->start_global_locating();
    pthread_create(&global_amcl_locate_thread, NULL, global_amcl_locate_thread_function, this);
}
void *AGVInstance::global_amcl_locate_thread_function(void *param)
{
    AGVInstance *ptr = (AGVInstance *)param;
    ptr->do_amcl_global_locating();

    return nullptr;
}
void AGVInstance::do_amcl_global_locating()
{
	scan_matching_pause = true; //先暂停跟踪定位的scan matching
	while(amcl_global_locate->first_located==false)
	{////等待amcl定位完成
		usleep(20000);
	}
	Position max_pose;
	max_pose.timestamp = get_current_time_us();
	max_pose.x = amcl_global_locate->locate_pose.x;
	max_pose.y = amcl_global_locate->locate_pose.y;
	max_pose.theta = amcl_global_locate->locate_pose.theta;
	update_last_position(max_pose);
	flush_odom_data();
	pose2d = max_pose;
	flush_odom_data();
	usleep(10000);


	set_raw_odom_position(max_pose);

	Position pose = expolate_current_position();
	printf("pose.x=%f,y=%f,theta=%f\n",pose.x,pose.y,pose.theta);
	scan_matching_pause = false;
	pthread_mutex_lock(&mutex_state);
	state = SYSTEM_STATE_FREE;
	pthread_mutex_unlock(&mutex_state);
	init_locate();
    pose_initilized = true;
}
void *AGVInstance::global_locate_thread_function(void *param)
{
    AGVInstance *ptr = (AGVInstance *)param;
    ptr->do_global_locating();
    return nullptr;
}
void * AGVInstance::pallet_recognize_thread_function(void * param)
{
	AGVInstance *ptr = (AGVInstance *)param;
	ptr->do_pallet_recognize();
	return nullptr;

}
void AGVInstance::do_pallet_recognize()
{
	long time_pre;
	int rec_sult;
	Position resultpose;
	while(1)
	{
	time_pre = get_current_time_us();
    rec_sult = pallet_recognize(1.25,0.8,180,20,resultpose);
	printf("time consume=%d  ",get_current_time_us()-time_pre);
	printf("result_pose pos x=%f,y=%f,theta=%f\n",resultpose.x,resultpose.y,resultpose.theta);
	usleep(5000000);
	}
}
//1.25,0.85,150,20
int AGVInstance::pallet_recognize(float len,float width,float angle_sech,int clc_cnt,Position & result_pose)////托盘位姿识别
{
	int rec_sult;
	rec_sult = pallet_pose_recgnize->pallet_recognize(len,width,angle_sech,clc_cnt,result_pose);
	return rec_sult;
}
//1.25,0.85,180,20
int AGVInstance::legs_recognize(float len,float width,float angle_sech,int clc_cnt,Position & leg1_pose,Position & leg2_pose)////托盘退部识别
{
	int rec_sult;
	rec_sult = pallet_pose_recgnize->legs_recognize(len,width,angle_sech,clc_cnt,leg1_pose,leg2_pose);
	return rec_sult;

}
int AGVInstance::start_path(vector <Position> pathpoints,vector <float> vel_points,vector <char> inverset,char doact)////ret -1 failed; 1 success;doact(0-no action,1-up,2-down)

{
    if(!pose_initilized) return -2;/////if robot pos is not initialized , return -2;
	int rec_sult;
	rec_sult = my_runcurve->initial_path(pathpoints,vel_points,inverset,doact);
	return rec_sult;

}

bool AGVInstance::cancel_path()
{
    //if(!pose_initilized) return false;/////if robot pos is not initialized , return false;
	bool rec_sult;
	rec_sult = my_runcurve->cancel_path();
	return rec_sult;

}
bool AGVInstance::stop_path()
{
	bool rec_sult;
	rec_sult = my_runcurve->stop_path();
	return rec_sult;

}
bool AGVInstance::continue_path()
{
	bool rec_sult;
	rec_sult = my_runcurve->continue_path();
	return rec_sult;

}

void AGVInstance::do_global_locating()
{
    scan_matching_pause = true; //先暂停跟踪定位的scan matching
    printf("Start global locating\n");

    //读取配置文件
    GET_CONFIG_DOUBLE(intensity_threshold, "radar", "intensity_threshold");
    GET_CONFIG_DOUBLE(max_distance, "scan_matching", "max_distance");
    GET_CONFIG_DOUBLE(occupied_threshold, "map", "occupied_threshold");

    GET_CONFIG_INT(iterations_count, "global_locating", "iterations_count");
    GET_CONFIG_DOUBLE(grid_resolution, "global_locating", "grid_resolution");
    GET_CONFIG_INT(particles_per_grid, "global_locating", "particles_per_grid");
    GET_CONFIG_INT(rotation_sample_count, "global_locating", "rotation_sample_count");
    GET_CONFIG_INT(rotation_time_s, "global_locating", "rotation_time_s");
    GET_CONFIG_INT(simulate_radar_data_size, "global_locating", "simulate_radar_data_size");
    GET_CONFIG_DOUBLE(simulate_radar_range, "global_locating", "simulate_radar_range");

    GET_CONFIG_DOUBLE(landmark_point_rate, "motecarlo", "landmark_point_rate");
    GET_CONFIG_DOUBLE(landmark_weight, "motecarlo", "landmark_weight");

    GET_CONFIG_DOUBLE(agv_size_x, "agv", "size_x");
    GET_CONFIG_DOUBLE(agv_size_y, "agv", "size_y");

    //全局定位代码从这里开始
    //建立周围环境的临时地图
    printf("Generate surrounding map...\n");
    char carto_dir[200];
    get_configs()->get_string("carto", "config_files_dir", carto_dir, nullptr);
    char carto_name[200];
    get_configs()->get_string("carto", "config_file_name", carto_name, nullptr);
    CartoModuleOption opt(carto_dir, carto_name);
    opt.radar_scan_time = get_configs()->get_float("carto", "radar_scan_time", nullptr);
    opt.pose_mathching_score = get_configs()->get_float("carto", "pose_matching_score", nullptr);
    opt.msg_queue = message_queue;
    opt.pose_channel = CHANNEL_POSE;
    opt.imu_channel = CHANNEL_IMU;
    opt.radar_channel = CHANNEL_RADAR;
    CartoModule *carto_tmp = new CartoModule(opt);
    usleep(500000);
    do_random_rotate = true;
    //启动旋转线程
    pthread_create(&random_rotate_thread, nullptr, random_rotate_thread_function, this);
    usleep(rotation_time_s * 1000000);
    do_random_rotate = false;
    carto_tmp->stop_and_optimize();
    std::vector<char> map_data;
    carto_tmp->paint_map(&map_data);
    SimpleGridMap *local_map = new SimpleGridMap(map_data.data(), map_data.size());	//转完出的图
    delete carto_tmp;

    // For DEBUG
    // local_map->binarization(occupied_threshold);
    // map_data.clear();
    // local_map->to_char_array(&map_data);
    std::ofstream ofile("global_locating_map");
    ofile.write(map_data.data(), map_data.size());
    ofile.close();

    //地图相关
    MapSharedPointer p_map;
    p_map = get_current_map();
    MapInfo info = p_map->get_info();
    double min_x = info.origen_x, max_x = min_x + info.resolution * info.width;
    double min_y = info.origen_y, max_y = min_y + info.resolution * info.height;
    DistanceMap dmap(p_map.get());
    DistanceMap dmap_landmark(p_map->get_landmark_map());

    //随机数相关
    std::default_random_engine random_e(get_current_time_us());
    std::uniform_real_distribution<double> distribution_x(min_x, max_x);
    std::uniform_real_distribution<double> distribution_y(min_y, max_y);
    std::uniform_real_distribution<double> distribution_theta(-3.141592653589763, 3.141592653589763);

    //将临时地图转化为模拟点云数据
    PointCloudData sim_radar_data(1);
    PointCloudData sim_landmark_data(1);
    double dist_sum = 0.;
    double min_dist = simulate_radar_range;
    double theta_step = 3.14159265 * 2. / simulate_radar_data_size;
    for (double t = 0; t < 3.14159265 * 2; t += theta_step)
    {
        double c = cos(t);
        double s = sin(t);
        bool get_radar = false;
        bool get_landmark = false;
        for (double d = 0.01; d < simulate_radar_range; d += 0.01)
        {
            double x = d * c;
            double y = d * s;
            double v = local_map->get(x, y);
            if (!get_radar && v > occupied_threshold)
            {
                sim_radar_data.add_point(x, y, 0.5);
                dist_sum += d;
                if (d < min_dist)
                {
                    min_dist = d;
                }
                get_radar = true;
            }
            v = local_map->get_landmark_map()->get(x, y);
            if (!get_landmark && v > occupied_threshold)
            {
                sim_landmark_data.add_point(x, y, 1);
                get_landmark = true;
            }
            if (get_radar && get_landmark)
            {
                break;
            }
        }
    }
    delete local_map;
    printf("Local map point cloud : %d points, %d landmark points, average distance %.3f\n", sim_radar_data.points.size(), sim_landmark_data.points.size(), dist_sum / sim_radar_data.points.size());
    bool use_landmark = (sim_landmark_data.points.size() > (sim_radar_data.points.size() * landmark_point_rate));

    double agv_r = ((agv_size_x > agv_size_y) ? agv_size_x : agv_size_y) / 2.;
    double no_point_area_r = min_dist * 0.8;
    double unreachable_r = ((agv_r > no_point_area_r) ? agv_r : no_point_area_r);
    int unreachable_range = (int)(unreachable_r / info.resolution) + 1;
    DistanceMap unreachable(p_map.get(), unreachable_range);

    //随机采样，对采样点优化后，计算匹配程度，记录得分最高的点
    double max_score = -0.1;
    Position max_pose;
    long start_time = get_current_time_us();
    printf("Start");
    int grid_count = 0;
    int grid_number = (int)((max_x - min_x) / grid_resolution + 1) * (int)((max_y - min_y) / grid_resolution + 1);
    int sample_count = 0;
    theta_step = 3.1415926 * 2. / rotation_sample_count;
	////////////////////////////////20201224lwg修改
	int midex, nidex;
	std::vector<double> scores;
	std::vector<Position> positions;
	scores.reserve(10);
	positions.reserve(10);
	for (midex = 0; midex < 10;midex++)
	{
		scores.push_back(0);
		positions.push_back(max_pose);
	}
	///////////////////////////////////
    for (double x = min_x; x < max_x; x += grid_resolution)
    {
        for (double y = min_y; y < max_y; y += grid_resolution)
        {
            printf("\rScanning block %d/%d...", ++grid_count, grid_number);
            for (int i = 0; i < particles_per_grid; i++)
            {
                Position p_sample(1, x + distribution_x(random_e), y + distribution_y(random_e), 0);
                if (unreachable.get(p_sample.x, p_sample.y) > 0)
                {
                    continue;
                }
                double v = p_map->get(p_sample.x, p_sample.y);
                if (v > -0.1 && v < occupied_threshold)
                {
                    for (double theta = 0; theta < 3.1415926 * 2; theta += theta_step)
                    {
                        Position p_rand(1, p_sample.x, p_sample.y, theta);
                        sample_count++;
                        //优化
                        p_rand = do_scan_matching(sim_radar_data, p_rand, "global_locating");
                        if (use_landmark)
                        {
                            p_rand = do_scan_matching(sim_landmark_data, p_rand, "global_locating");
                        }
                        //计算得分
                        double score = 0.;
                        for (auto &point : sim_radar_data.points)
                        {
                            Position p_local(1, point(0), point(1), 0.);
                            Position p_global = p_rand * p_local;
                            double v2 = dmap.get(p_global.x, p_global.y);
                            if (v2 < 0)
                            {
                                v2 = 0;
                            }
                            score += v2;
                        }
                        score = score / sim_radar_data.points.size();
                        if (use_landmark)
                        {
                            double score_landmark = 0.;
                            for (auto &point : sim_landmark_data.points)
                            {
                                Position p_local(1, point(0), point(1), 0.);
                                Position p_global = p_rand * p_local;
                                double v2 = dmap_landmark.get(p_global.x, p_global.y);
                                if (v2 < 0)
                                {
                                    v2 = 0;
                                }
                                score_landmark += v2;
                            }
                            score_landmark = score_landmark / sim_landmark_data.points.size();
                            score = (score + score_landmark * landmark_weight) / (landmark_weight + 1);
                        }
						//if (score > max_score)
						//{
						//	max_score = score;
						//	max_pose = p_rand;
						//}
						for ( midex = 0; midex < 10;midex++)
						{
							if (score > scores[midex])
                        {
								for (nidex = 9; nidex > midex; nidex--)
								{
									scores[nidex] = scores[nidex-1];
									positions[nidex] = positions[nidex - 1];
								}
								scores[midex] = score;
								positions[midex] = p_rand;
								break;
							}

                        }
                    }
                }
            }
        }
    }
    printf("\nDone, %d samples, use time %.3fs\n", sample_count, (get_current_time_us() - start_time) / 1000000.);
	double pi = 3.1415926535;
	for (midex = 0; midex < 10;midex++)
	{
		if (positions[midex].theta>pi) positions[midex].theta -= 2 * pi;
		if (positions[midex].theta<-pi) positions[midex].theta += 2 * pi;
	}
    //叠加上全局定位过程中的位姿变化量，优化得到最终结果
	//printf("Scanning result (%.3f, %.3f, %.3f), score %.3f\n", max_pose.x, max_pose.y, max_pose.theta, max_score);
	int max_theta_idex=0;
	int max_theta_cnt=0;
	int theta_cnt = 0;
	for (midex = 0; midex < 10;midex++)
	{
		
		theta_cnt = 0;
		for (nidex = 0; nidex < 10;nidex++)
		{
			if (fabs(positions[midex].theta - positions[nidex].theta) < 0.2) theta_cnt++;
		}
		printf("Scanning result (%.3f, %.3f, %.3f), score %.3f,similor_cnt=%d\n", positions[midex].x, positions[midex].y, positions[midex].theta, scores[midex], theta_cnt);
		if (theta_cnt > max_theta_cnt){ max_theta_cnt = theta_cnt; max_theta_idex = midex; }
	}
	
	double mscore = scores[max_theta_idex];
	max_pose = positions[max_theta_idex];
	for (nidex = 0; nidex < 10;nidex++)
	{
		if (fabs(positions[max_theta_idex].theta - positions[nidex].theta) < 0.2)
		{
			if (scores[nidex]>mscore ){ max_pose = positions[nidex];mscore =scores[nidex];}
		}
	}
	printf("Scanning result (%.3f, %.3f, %.3f), score %.3f\n", max_pose.x, max_pose.y, max_pose.theta, mscore );

    Position pose_change = get_last_position();
    max_pose = max_pose * pose_change;
    PointCloudData radar_data = get_last_radar_data();
    PointCloudData landmark_data = radar_data.intercept(intensity_threshold);
    max_pose = do_scan_matching(radar_data, max_pose, "scan_matching");
    if (landmark_data.points.size() > (radar_data.points.size() * landmark_point_rate))
    {
        max_pose = do_scan_matching(landmark_data, max_pose, "landmark_scan_matching");
    }
    printf("Global locating result (%.3f, %.3f, %.3f)\n", max_pose.x, max_pose.y, max_pose.theta);
	//max_pose.timestamp = radar_data.timestamp;
	max_pose.timestamp = get_current_time_us();

	update_last_position(max_pose);
	flush_odom_data();
	pose2d = max_pose;
	flush_odom_data();
	usleep(10000);
	Position pose = expolate_current_position();
	printf("pose.x=%f,y=%f,theta=%f\n",pose.x,pose.y,pose.theta);
    scan_matching_pause = false;
    pthread_mutex_lock(&mutex_state);
    state = SYSTEM_STATE_FREE;
    pthread_mutex_unlock(&mutex_state);
	init_locate();

};
void *AGVInstance::random_rotate_thread_function(void *param)
{
    AGVInstance *ptr = (AGVInstance *)param;
    GET_CONFIG_INT(rotation_speed, "global_locating", "rotation_speed");
    GET_CONFIG_LONG(rotation_message_frequency_us, "global_locating", "rotation_message_frequency_us");
    MoveInstruction move;
    move.type = MOVE_INSTRUCTION_TYPE_SPEED;
    move.speed.speed = 0;
    move.speed.turn_left = rotation_speed;
    std::vector<char> data;
    move.to_char_array(&data);
    ptr->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
    while (ptr->do_random_rotate)
    {
        usleep(rotation_message_frequency_us);
    }
    move.speed.turn_left = 0;
    data.clear();
    move.to_char_array(&data);
    ptr->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
}

Position AGVInstance::do_scan_matching(PointCloudData &data, Position &pose, char *config_item)
{
    MapSharedPointer p_map;
    p_map = get_current_map();
    doing_scan_matching = true;

    float min_dist = get_configs()->get_float(config_item, "min_distance", nullptr);
    float max_dist = get_configs()->get_float(config_item, "max_distance", nullptr);

    min_dist = min_dist * min_dist;
    max_dist = max_dist * max_dist;
    int size = data.points.size();
    PointCloudData points(data.timestamp, size);

    for (int i = 0; i < size; i++)
    {
        double x = data.points[i](0);
        double y = data.points[i](1);
        double dist = x * x + y * y;
        if (dist > min_dist && dist < max_dist)
        {
            points.add_point(x, y, 1);
        }
    }

    ScanMatchingOptions opt;
    opt.occupied_space_cost_factor = get_configs()->get_float(config_item, "occupied_space_cost_factor", nullptr);
    opt.translation_delta_cost_factor = get_configs()->get_float(config_item, "translation_delta_cost_factor", nullptr);
    opt.rotation_delta_cost_factor = get_configs()->get_float(config_item, "rotation_delta_cost_factor", nullptr);
    opt.num_threads = get_configs()->get_int(config_item, "num_threads", nullptr);
    opt.max_num_iterations = get_configs()->get_int(config_item, "max_num_iterations", nullptr);
	Position optimism;
	/*char* str_gl = "global_locating";
	if(strcmp(config_item,str_gl)==0)
	{
		//std::cout<<"low start:\n"<<std::endl;
		//Low_resolution my_lowreso(p_map.get(),occupied_threshold);
		optimism = scan_matching(pose, low_reso_map, data, opt);

	}
	else*/
	    optimism = scan_matching(pose, p_map.get(), data, opt);
    optimism.timestamp = data.timestamp;

    doing_scan_matching = false;

    return optimism;
}

//void AGVInstance::do_scan_matching(PointCloudData &data)
void AGVInstance::do_scan_matching(PointCloudData &data, Position &pos_ret ,bool &is_correct)
{
    MapSharedPointer p_map;
    p_map = get_current_map();
    if (p_map.is_nullptr() || doing_scan_matching || scan_matching_pause)
    {
        // printf("Skip scan matching\n");
	is_correct = false;
        return;
    }

    float intensity_threshold = get_configs()->get_float("radar", "intensity_threshold", nullptr);
    float landmark_point_rate = get_configs()->get_float("landmark_scan_matching", "point_rate", nullptr);
    // int multi_landmark_detect = get_configs()->get_int("landmark_scan_matching", "multi_landmark_detect", nullptr);
    int size = data.points.size();
    PointCloudData landmark_points(data.timestamp, size);
    for (int i = 0; i < size; i++)
    {
        double x = data.points[i](0);
        double y = data.points[i](1);
        if (data.intensities[i] > intensity_threshold)
        {
            landmark_points.add_point(x, y, 1);
        }
    }

    //Position pose = expolate_current_position_tim(data.timestamp);
    Position pose = expolate_current_position();
    Position optimism = do_scan_matching(data, pose, "scan_matching");
    Position optimism_mk = optimism;
    if (landmark_points.points.size() > size * landmark_point_rate)
    {
        optimism_mk = do_scan_matching(landmark_points, optimism, "landmark_scan_matching");
	fprintf(fp_scanmatching_pos,"%ld	%d	%f	%f	%f\n",optimism_mk.timestamp,2,optimism_mk.x,optimism_mk.y,optimism_mk.theta);
	optimism = optimism_mk;			

    }

	is_correct = true;
	pos_ret = optimism;
	
    std::vector<char> sendbuf;
    optimism.to_char_array(&sendbuf);
    message_queue->send(CHANNEL_POSE, sendbuf.data(), sendbuf.size());
};
bool AGVInstance::savePoseToServer()/////lwg20201218将机器人实时位姿写入到文件
{
	if (get_system_state() == SYSTEM_STATE_MAPPING) return false;//////机器人建图过程中不记录位姿数据
	if((!scan_init) || (on_scan_matching_cnt_<50)) return false;/////机器人初始化未完成，不记录位姿数据
	Position pose_odom;
	pose_odom = expolate_current_position();///////获取当前时刻机器人的位姿数据
	pose_from_save_file = pose_odom;
	FILE *IniFile;
	IniFile = fopen("zero.ini", "w");
	//char* buf;
	if (IniFile != NULL)
	{
		fprintf(IniFile,"%s %f\n","initial_pose_x=",pose_odom.x);
		fprintf(IniFile,"%s %f\n","initial_pose_y=",pose_odom.y);
		fprintf(IniFile,"%s %f\n","initial_pose_a=",pose_odom.theta);	
		//printf("写入文件中的初始位姿为   %f   %f   %f\n",pose_odom.x,pose_odom.y,pose_odom.theta);	
	}
	else printf("文件打开失败");
	fclose(IniFile);
	return true;
}
bool AGVInstance::loadPoseFromserver()/////lwg20201218将机器人位姿从文件中加载
{
	Position pose_odom;////从文件中读取保存的位姿
	pose_odom.timestamp = get_current_time_us();
	char szTest[1000]={0};
	char *temp=NULL;
	FILE *IniFile;
	IniFile= fopen("zero.ini", "r");
	if (IniFile != NULL)
	{
	while(!feof(IniFile))
		{
			memset(szTest, 0, sizeof(szTest)); 
			 fgets(szTest, sizeof(szTest) - 1, IniFile); // 包含了\n
			if (strstr(szTest, "initial_pose_x=") != NULL)
			{
			temp = My_strsub_i2e(szTest,16,10);
			pose_odom.x = atof(temp);///初始位姿均值（x）.
			free(temp);
			}
			else if (strstr(szTest, "initial_pose_y=") != NULL)
			{
			temp = My_strsub_i2e(szTest,16,10);
			pose_odom.y = atof(temp);///初始位姿均值（y）.
			free(temp);
			}
			else if (strstr(szTest, "initial_pose_a=") != NULL)
			{
			temp = My_strsub_i2e(szTest,16,10);
			pose_odom.theta = atof(temp);///初始位姿均值（x）.
			free(temp);
			}
		}	
	}
	fclose(IniFile);
	pose_from_save_file = pose_odom;
	printf("从文件中加载的初始位姿为   %f   %f   %f\n",pose_odom.x,pose_odom.y,pose_odom.theta);
	update_last_position(pose_odom);
	return true;
}

bool AGVInstance::loadPose(char* src)
{
	Position pose_odom;////从文件中读取保存的位姿
	pose_odom.timestamp = get_current_time_us();
	double *init_position = (double*)src;
	if((fabs(init_position[0])>fuzzy_1)||(fabs(init_position[1])>fuzzy_1))
	{
	pose_odom.x = init_position[0];
	pose_odom.y = init_position[1];
	pose_odom.theta = init_position[2];
	}
	else
	{
	pose_odom.x = pose_from_save_file.x;
	pose_odom.y = pose_from_save_file.y;
	pose_odom.theta = pose_from_save_file.theta;
	}
	printf("接收初始位姿为   %f   %f   %f\n",pose_odom.x,pose_odom.y,pose_odom.theta);
	//update_last_position(pose_odom);
    pthread_mutex_lock(&mutex_state);
    if (state == SYSTEM_STATE_FREE)
    {
        state = SYSTEM_STATE_INIT_POSE;
    }
    else
    {
        pthread_mutex_unlock(&mutex_state);
        return false;
    }
    pthread_mutex_unlock(&mutex_state);
    MapSharedPointer p_map;
    p_map = get_current_map();
	
	amcl_global_locate->gridmap = p_map.get();
    amcl_global_locate->set_init_pos(pose_odom.x,pose_odom.y,pose_odom.theta);
	amcl_global_locate->start_global_locating();
    pthread_create(&global_amcl_locate_thread, NULL, global_amcl_locate_thread_function, this);
	return true;
}

//--------------------------------------------------------------------

AGVInstance *instance = nullptr;

void init_global_agv_instance()
{
    if (instance == nullptr)
    {
        instance = new AGVInstance();
    }
}

AGVInstance *get_global_agv_instance()
{
    return instance;
}

storage *get_global_storage()
{
    return instance->st;
}