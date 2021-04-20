#include <map>
#include <iostream>
#include <cmath>
#include <unistd.h>

#include "common/data_transform.h"
#include "common/configuration.h"
#include "nav.h"
#include "agv.h"
/*
int start_path(vector <Position> pathpoints,vector <float> vel_points,vector <char> inverset);////ret -1 failed; 1 success;
bool cancel_path();

bool stop_path();
bool continue_path();
*/

NavModule::NavModule(NavModuleOption &opt): option(opt) {
	
    pthread_mutex_init(&sensor_mutex, nullptr);
    if(option.msg_queue != nullptr) {
        pose_listener_id = option.msg_queue->add_listener(option.pose_channel, this);
        arrive_listener_id = option.msg_queue->add_listener(option.arrive_channel, this);
    }
	chain=option.chain;
	caproute=chain.routes.front();
	get_global_agv_instance()->exec_route=caproute.id;
	
	
	for(CapPoint &point : caproute.cap_points) {
		printf("id=%d,x=%lf,y=%lf,dir=%d\n",point.id,point.x,point.y,point.dir);
	}
	printf("ops.size=%d\n",caproute.op_point.ops.size());
	for(Operation &op : caproute.op_point.ops) {
		printf("op.type=%d\n",op.type);
	}
	cap_path_planning_new_if(caproute);
};

NavModule::~NavModule() {
    pthread_mutex_destroy(&sensor_mutex);
    if(option.msg_queue != nullptr) {
        option.msg_queue->remove_listener(pose_listener_id);
        option.msg_queue->remove_listener(arrive_listener_id);
    }
};
#define PI (3.141592653589793)

bool line_moving=false;


//在路径尾部绑一个动作
void NavModule::cap_path_planning_new_if(CapRoute& cr){
	std::list<CapPoint>::iterator itc;
	vector<Position> caps;
	vector<float>	speed;
	vector<char>	dir;
	
	Position pose = get_global_agv_instance()->expolate_current_position();
	CapPoint start(0,pose.x,pose.y,pose.theta);
	CapPoint target;
	
	for(itc=cr.cap_points.begin();itc!=cr.cap_points.end();itc++){
		
		target = *itc;
		target.theta=atan2(target.y - start.y,target.x - start.x);
		if(target.dir!=0){
			target.theta-=PI;
		}
		
		caps.push_back(Position(0,itc->x,itc->y,target.theta));
		speed.push_back(0.5f);
		dir.push_back((char)itc->dir);
		start=target;
	}
	char arg=0;
	switch(cr.op_id){
		case 0:arg=1;break;
		case 1:arg=2;break;
		case 0xff:arg=0;break;
		default:break;
	}
//	int start_path(vector <Position> pathpoints,vector <float> vel_points,vector <char> inverset);////ret -1 failed; 1 success;
//(0-no action,1-up,2-down)
	get_global_agv_instance()->start_path(caps,speed,dir,arg);
}



//在路径尾部绑一个动作
void NavModule::cap_path_planning2(CapRoute& cr){
	actions.clear();
	
	Position pose = get_global_agv_instance()->expolate_current_position();
	printf("##############cap_path_plan###############\n");
	printf("start:x=%.3lf,y=%.3lf,theta=%.3lf\n",pose.x,pose.y,pose.theta);
	
	CapPoint start(0,pose.x,pose.y,pose.theta);
	CapPoint target = cr.cap_points.front();
	get_global_agv_instance()->exec_cap=target.id;
	
	printf("target:x=%.3lf,y=%.3lf,theta=%.3lf,dir=%d\n",target.x,target.y,target.theta,target.dir);
	
	//先计算转向
	target.theta=atan2(target.y - start.y,target.x - start.x);
	
	if(target.collision==1){//开避障
		get_global_agv_instance()->collision_switch=true;
	}else{
		get_global_agv_instance()->collision_switch=false;
		get_global_agv_instance()->collision_level=0;
		MoveInstruction move;
		move.type = MOVE_INSTRUCTION_TYPE_COLLISION;
		move.collision.level=0;
		std::vector<char> data;
		move.to_char_array(&data);
		get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
	}
	
	if(target.dir!=0){
		target.theta-=PI;
		printf("####dir==1\n");
	}
	double dtheta=target.theta-start.theta;
	while((dtheta>PI)||(dtheta<-PI)){	//转向控制在180度以内
		if(dtheta>PI){
			dtheta-=2*PI;
		}else if(dtheta<-PI){
			dtheta+=2*PI;
		}
	}
	printf("");
	actions.push_back(Action_clm(	start.x,start.y,start.theta,
								target.x,target.y,target.theta,
								dtheta));
	actions.push_back(Action_clm(	start.x,start.y,target.theta,//转之后的
								target.x,target.y,target.theta,
								target.x-start.x,target.y-start.y,target.dir));
	
	double val=180/PI;
	int i=0;
	std::list<Action_clm>::iterator itc;
	for(itc=actions.begin();itc!=actions.end();itc++){
		Action_clm act = *itc;
		if(act.type==1){
			printf("action%d,line,from  %.3lf  %.3lf  %.3lf,to  %.3lf  %.3lf  %.3lf,  %.3lf  %.3lf  dir=%d\n",i,
			act.from_x,act.from_y,act.from_theta*val,
			act.to_x,act.to_y,act.to_theta*val,
			act.x,act.y,act.dir);
		}else{
			printf("action%d,turn,from  %.3lf  %.3lf  %.3lf,to  %.3lf  %.3lf  %.3lf,  %.3lf  %.3lf  %.3lf\n",i,
			act.from_x,act.from_y,act.from_theta*val,
			act.to_x,act.to_y,act.to_theta*val,
			act.x,act.y,act.angle*val);
		}
		i++;
	}
	printf("############end cap_path_plan#############\n");
	Action_clm action=actions.front();
	if(action.type==0){
		send_turn_msg(action.angle>=0?7:8,action.angle>=0?action.angle*val:0-action.angle*val);
	}
}


void NavModule::recieve_message(const int channel, char *buf, const int size) {
    if (channel == CHANNEL_POSE)
    {
        Position pose;
        pose.from_char_array(buf, size);
		
    }else if (channel == CHANNEL_ARRIVE)
    {
		Position pose = get_global_agv_instance()->expolate_current_position();
		double val=180/PI;
		double angle=pose.theta*val;
		while(angle<=-180.){
			angle+=360;
		}while(angle>180.){
			angle-=360;
		}
		printf("###NavModule::CHANNEL_ARRIVE:x=%.3lf,y=%.3lf,theta=%.3lf\n",pose.x,pose.y,angle);
		
		int cmd=*((int*)buf);
		switch(cmd){
			case 0xc101://上报点对点到达
				printf("跑完最后一个nav点\n");
				/*
				operation_finished=false;
				pthread_create(&handle_operation_thread, NULL, handle_operation_thread_function, this);
				
				printf("###opp.ops.size()=%d\n",caproute.op_point.ops.size());
				while(!operation_finished){
					usleep(100*1000);
				}*/
				chain.routes.pop_front();
				//检查是否有剩余route
				//如果有,则切换到下一个route,如果没有,结束
				if(chain.routes.size()==0){
					printf("stop_navigating()\n");
					get_global_agv_instance()->stop_navigating();
					
				}else{
					caproute=chain.routes.front();
					get_global_agv_instance()->exec_route=caproute.id;
					
					for(CapPoint &point : caproute.cap_points) {
						printf("id=%d,x=%lf,y=%lf,dir=%d\n",point.id,point.x,point.y,point.dir);
					}
					printf("ops.size=%d\n",caproute.op_point.ops.size());
					for(Operation &op : caproute.op_point.ops) {
						printf("op.type=%d\n",op.type);
					}
					
					cap_path_planning_new_if(caproute);
				}
				break;
			case 0xc103://上报顶升上升完成
				printf("上报顶升上升完成\n");
				lift_up_finished=true;
				break;
			case 0xc104://上报顶升下降完成
				printf("上报顶升下降完成\n");
				lift_down_finished=true;
				break;
			default:
				break;
		}
    }
}
void *NavModule::handle_operation_thread_function(void *param){
	
	NavModule *ptr = (NavModule *)param;
	
	OperationPoint opp=ptr->caproute.op_point;
	
	get_global_agv_instance()->exec_cap=0xffff;
	while(opp.ops.size()){
		Operation op=opp.ops.front();
		switch(op.type){
			case OP_WAIT_DI:
//				while(!op.wait.flag){
					usleep(1000*100);
//				}
				printf("OP_WAIT_DI finish\n");
				break;
			
			case OP_DELAY:
				printf("OP_DELAY\n");
				sleep(op.delay.sec);
				break;
			case OP_LIFT_UP:
				printf("OP_LIFT_UP\n");
				ptr->send_lift_msg(op.lift.dir);
				while(!ptr->lift_up_finished){
					usleep(1000*100);
				}
				ptr->lift_up_finished=false;
				printf("OP_LIFT_UP finish\n");
				break;
			case OP_LIFT_DOWN:
				printf("OP_LIFT_DOWN\n");
				ptr->send_lift_msg(op.lift.dir);
				while(!ptr->lift_down_finished){
					usleep(1000*100);
				}
				ptr->lift_down_finished=false;
				printf("OP_LIFT_DOWN finish\n");
				break;
			case OP_RADAR:
				{
					Position pose1,pose2;
					printf("OP_RADAR\n");
					get_global_agv_instance()->masks.clear();
					
					//这里调用李工算法,获取桌子腿中心坐标和半径
//					get_global_agv_instance()->legs_recognize(1.25,0.8,180,20,pose1,pose2);
					get_global_agv_instance()->legs_recognize(0.8,1.25,170,20,pose1,pose2);
					
					get_global_agv_instance()->masks.push_back(pose1);
					get_global_agv_instance()->masks.push_back(pose2);
					get_global_agv_instance()->mask_radius=0.05;//腿半径设为0.05m
				}
				break;
			case OP_MASK:
				printf("OP_MASK\n");
				get_global_agv_instance()->mask_switch=op.mask.onoff;
				break;
			default:
				break;
		}
		opp.ops.pop_front();
	}
	get_global_agv_instance()->collision_switch=true;
	ptr->operation_finished=true;
}
void NavModule::send_lift_msg(int dir){
	MoveInstruction move;
	move.type = MOVE_INSTRUCTION_TYPE_LIFT;
	move.lift.dir=dir;
	std::vector<char> data;
	move.to_char_array(&data);
	get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
}

void NavModule::send_turn_msg(int dir, double angle){
	MoveInstruction move;
	move.type = MOVE_INSTRUCTION_TYPE_TURN;
	move.turn.dir=dir;
	move.turn.angle=angle;
	std::vector<char> data;
	move.to_char_array(&data);
	get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
}

void NavModule::send_nav_msg(double start_x, double start_y,double start_theta,double end_x,double end_y,int dir){
	MoveInstruction move;
	move.type = MOVE_INSTRUCTION_TYPE_TARGET;
	move.target.start_x=start_x;
	move.target.start_y=start_y;
	move.target.start_theta=start_theta;
	move.target.x=end_x;
	move.target.y=end_y;
	move.target.theta=start_theta;//todo
	move.target.dir=dir;
	std::vector<char> data;
	move.to_char_array(&data);
	get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
}

int NavModule::handle_radar_data(PointCloudData & data) {
    pthread_mutex_lock(&sensor_mutex);
    pthread_mutex_unlock(&sensor_mutex);
    return 0;
};

int NavModule::handle_imu_data(ImuData2D & data) {
    pthread_mutex_lock(&sensor_mutex);
    pthread_mutex_unlock(&sensor_mutex);
    return 0;
};
