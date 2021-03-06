#include <vector>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>

#include "common/thpool.h"
#include "common/configuration.h"
#include "agv.h"
#include "storage.h"
#include "common/capture_point.h"

int local_port = 12345;
using namespace std;

int byte2int(char *buf)
{
    int value = 0;
    for (int i = 0; i < 4; i++)
    {
        value |= ((int)(buf[i] & 0xff)) << (8 * i);
    }
    return value;
}
void int2byte(char *buf, int value)
{
    for (int i = 0; i < 4; i++)
    {
        buf[i] = (char)((value >> 8 * i) & 0xff);
    }
}

void do_sha256(char *buf, int size, char *output)
{
    return;
}

#define HEAD_SIZE 24
class TCPDataPackage
{
public:
    TCPDataPackage(int func) : function(func){};
    TCPDataPackage(int func, int arg1, int arg2, int data_len) : function(func), arg1(arg1), arg2(arg2), data_size(data_len)
    {
        if (data_size > 0)
        {
            data_buf = (char *)malloc(data_size);
        }
    };
    TCPDataPackage(char *head, int *check_result)
    {
        function = head[4] & 0xff;
        arg1 = byte2int(head + 8);
        arg2 = byte2int(head + 12);
        data_size = byte2int(head + 16);
        //@Todo
        //check sum
        *check_result = 0;

        if (data_size > 0)
        {
            data_buf = (char *)malloc(data_size);
        }
    }
    ~TCPDataPackage()
    {
        if (data_buf)
        {
            free(data_buf);
        }
    }
    int function;
    int arg1 = 0;
    int arg2 = 0;
    int data_size = 0;
    char *data_buf = nullptr;

    void reserve_buf(int size)
    {
        if (size < 0)
        {
            return;
        }
        data_size = size;
        if (data_buf)
        {
            delete data_buf;
            data_buf = nullptr;
        }
        if (data_size > 0)
        {
            data_buf = (char *)malloc(data_size);
        }
    }

    void to_bytes(char *buf)
    {
        int timestamp = (int)(get_current_time_us() / 1000000);
        int2byte(buf, timestamp);
        buf[4] = (char)function;
        int2byte(buf + 8, arg1);
        int2byte(buf + 12, arg2);
        int2byte(buf + 16, data_size);
        //@Todo
        //check sum
    }
};

void handle_package(TCPDataPackage *input, TCPDataPackage *output)
{
    int func = input->function;
    if(func != 2) {
        printf("Recieve function %d with %d bytes data\n", func, input->data_size);
    }
    if (func == 0)//??????
    {
        MoveInstruction move;
        move.type = MOVE_INSTRUCTION_TYPE_SPEED;
        move.speed.speed = 0;
        move.speed.turn_left = 0;
        std::vector<char> data;
        move.to_char_array(&data);
        get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
    }
    else if (func == 1)//??????
    {
        MoveInstruction move;
        move.type = MOVE_INSTRUCTION_TYPE_SPEED;
        move.speed.speed = input->arg1;
        move.speed.turn_left = input->arg2;
        std::vector<char> data;
        move.to_char_array(&data);
        printf("move %d, %d\n", input->arg1, input->arg2);
        get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
    }
    else if (func == 2)//????????????
    {
        Position pose = get_global_agv_instance()->expolate_current_position();
        double score = 1;
        output->arg1 = get_global_agv_instance()->get_system_state();
        output->arg2 = get_global_agv_instance()->collision_level;
        output->reserve_buf(sizeof(double) * 4 + sizeof(int)*8);
		
		char *pc=output->data_buf;
        double *pd = (double *)pc;
        pd[0] = pose.x;
        pd[1] = pose.y;
        pd[2] = pose.theta;
        pd[3] = score;
		
		pc+=32;
        int * pi=(int *)pc;
		pi[0]=	get_global_agv_instance()->colli_cnt1;
		pi[1]=	get_global_agv_instance()->colli_cnt2;
		pi[2]=	get_global_agv_instance()->colli_cnt3;
        pi[3] = get_global_agv_instance()->mcu_input;
        pi[4] = get_global_agv_instance()->mcu_output;
        pi[5] = get_global_agv_instance()->mcu_battery_level;
        pi[6] = get_global_agv_instance()->mcu_speed;
        pi[7] = get_global_agv_instance()->mcu_omega;
		
//		printf("mcu_battery_level=%d,mcu_speed=%d,mcu_omega=%d\n",pi[5],pi[6],pi[7]);
    }
    else if (func == 3)//????????????
    {
        MapSharedPointer p_map;
        p_map = get_global_agv_instance()->get_current_map();
        if(! p_map.is_nullptr()) {
            std::vector<char> map_data;
            p_map->to_char_array(&map_data);
            int size = map_data.size();
            char *p = map_data.data();
            output->reserve_buf(size);
            memcpy(output->data_buf, p, size);
        }
    }
    else if (func == 129)//??????????????????
    {
        output->arg1 = get_global_agv_instance()->exec_route;
        output->arg2 = get_global_agv_instance()->exec_cap;
    }
    else if (func == 134)//??????cap
    {
		printf("###clm:rcv msg 134,??????cap\n");
		output->arg1=get_global_agv_instance()->caps.size();
		output->reserve_buf(get_global_agv_instance()->caps.size()*20);
		char* pc=output->data_buf;
		
		std::list<CapPoint>::iterator itc;
		for(itc=get_global_agv_instance()->caps.begin();itc!=get_global_agv_instance()->caps.end();itc++){
			CapPoint act = *itc;
			printf("##x=%lf,y=%lf,id=%d\n",act.x,act.y,act.id);
			double *pd = (double *)pc;
			pd[0] = act.x;
			pd[1] = act.y;
			pc+=16;
			int *pi=(int *)pc;
			pi[0]=act.id;
			pc+=4;
//			printf("@@x=%lf,y=%lf,id=%d\n",pd[0],pd[1],pi[0]);
		}
    }
	else if (func == 140)//???????????????
    {
		printf("###clm:rcv msg 140,???????????????\n");
		output->arg1=get_global_agv_instance()->routes.size();
		output->reserve_buf(get_global_agv_instance()->routes.size()*4);
		int* pi=(int *)output->data_buf;
		for(CapRoute &route : get_global_agv_instance()->routes) {
			printf("route %d:\n",route.id);
			pi[0]=route.id;
			pi++;
			int i=0;
			for(CapPoint &point : route.cap_points) {
				printf("	cap index=%d,id=%d,x=%lf,y=%lf,dir=%d\n",i,point.id,point.x,point.y,point.dir);
				i++;
			}
			i=0;
			printf("operation num= %d:\n",route.op_point.ops.size());
			for(Operation &op : route.op_point.ops) {
				printf("	op %d=%s\n",i,op.get_op_name());
				i++;
			}
		}
		printf("clm:leave msg 140,route_num=%d\n",get_global_agv_instance()->routes.size());
    }
	else if (func == 145)//????????????
    {
		printf("###clm:rcv msg 145,????????????\n");
		int id=input->arg1;
		int i=0;
		for(CapRoute &route : get_global_agv_instance()->routes) {
			if(route.id==id){
				printf("route %d:\n",route.id);
				
				output->arg1=route.cap_points.size();
				output->reserve_buf(route.cap_points.size()*20);
				char* pc=output->data_buf;
				
				for(CapPoint &point : route.cap_points) {
					printf("	cap index=%d,id=%d,x=%lf,y=%lf,dir=%d\n",i,point.id,point.x,point.y,point.dir);
					double *pd = (double *)pc;
					pd[0] = point.x;
					pd[1] = point.y;
					pc+=16;
					int *pi=(int *)pc;
					pi[0]=point.id;
					pc+=4;
					pc[0]=point.dir;
					pc[1]=point.collision;
					pc+=4;
					i++;
				}
			}
		}
		printf("clm:leave msg 145,route_num=%d\n",get_global_agv_instance()->routes.size());
    }
	else if (func == 149)//??????????????????
    {
		printf("###clm:rcv msg 149,??????????????????\n");
		output->arg1=get_global_agv_instance()->chains.size();
		output->reserve_buf(get_global_agv_instance()->chains.size()*4);
		int* pi=(int *)output->data_buf;
		for(RouteChain &chain : get_global_agv_instance()->chains) {
			printf("chain %d:\n",chain.id);
			pi[0]=chain.id;
			pi++;
			int i=0;
			for(CapRoute &route : chain.routes) {
				printf("	route index=%d,id=%d\n",i,route.id);
				i++;
			}
		}
		printf("clm:leave msg 149,chain_num=%d\n",get_global_agv_instance()->chains.size());
    }
	else if (func == 150)//???????????????
    {
		printf("###clm:rcv msg 150,???????????????\n");
		int id=input->arg1;
		int i=0;
		for(RouteChain &chain : get_global_agv_instance()->chains) {
			if(chain.id==id){
				printf("chain %d:\n",chain.id);
				
				output->arg1=chain.routes.size();
				output->reserve_buf(chain.routes.size()*20);
				char* pc=output->data_buf;
				
				for(CapRoute &route : chain.routes) {
					printf("	route index=%d,id=%d\n",i,route.id);
					int *pi=(int *)pc;
					pi[0]=route.id;
					pc+=4;					
					i++;
				}
			}
		}
		printf("clm:leave msg 150,route_num=%d\n",get_global_agv_instance()->routes.size());
    }
    else if (func == 143)//??????cloud
    {
		printf("###clm:rcv msg 143,??????cloud\n");
		output->arg1=get_global_agv_instance()->get_last_radar_data().points.size();
		output->reserve_buf(output->arg1*16);
		double* pc=(double*)output->data_buf;
		Position base=get_global_agv_instance()->expolate_current_position();
		
		printf("###now.x=%lf,y=%lf\n",base.x,base.y);
		std::vector<Eigen::Vector2d>::iterator itc;
		for(itc=get_global_agv_instance()->get_last_radar_data().points.begin();itc!=get_global_agv_instance()->get_last_radar_data().points.end();itc++){
			Eigen::Vector2d act = *itc;
			
			Position p=Position(0,act(0),act(1),0);
			p=base*p;
			pc[0] = p.x;
			pc[1] = p.y;
//			printf("##x=%lf,y=%lf\n",pc[0],pc[1]);
			pc+=2;
		}
    }
    else if (func == 141)//??????leg
    {
		printf("###clm:rcv msg 141,??????leg\n");
		output->arg1=2;
		output->reserve_buf(2*16);
		char* pc=output->data_buf;
		
		Position pose1,pose2;
		get_global_agv_instance()->masks.clear();
//		get_global_agv_instance()->legs_recognize(1.25,0.8,170,20,pose1,pose2);
		get_global_agv_instance()->legs_recognize(0.8,1.25,170,20,pose1,pose2);
		double radar_x = get_configs()->get_float("radar", "radar_position_x", nullptr);
		pose1.x+=radar_x;
		pose2.x+=radar_x;
		
		pose1 = get_global_agv_instance()->expolate_current_position()*pose1;
		pose2 = get_global_agv_instance()->expolate_current_position()*pose2;
		double *pd = (double *)pc;
		pd[0] = pose1.x;
		pd[1] = pose1.y;
		pd[2] = pose2.x;
		pd[3] = pose2.y;
    }
    else if (func == 17)//????????????
    {
        if(input->data_size > 0) {
            get_global_agv_instance()->load_grid_map(input->data_buf + sizeof(double)*2, input->data_size - sizeof(double)*2);
           // get_global_agv_instance()->start_global_locating();
           get_global_agv_instance()->loadPoseFromserver();
        }
    }
    else if (func == 117)//????????????2
    {
		printf("###clm:rcv msg 117,????????????2\n");
        if(input->data_size > 0) {
            get_global_agv_instance()->load_grid_map(input->data_buf + sizeof(double)*3, input->data_size - sizeof(double)*3);
			get_global_agv_instance()->loadPose(input->data_buf);
        }
    }
    else if (func == 18)//????????????
    {
        get_global_agv_instance()->start_mapping();
    }
    else if (func == 21)//????????????
    {
        get_global_agv_instance()->stop_mapping();
		//??????????????????,????????????
    }
	else if (func == 130)//??????cap???
    {
		printf("###clm:rcv msg 130,??????cap???\n");
		Position pose = get_global_agv_instance()->expolate_current_position();
		
		int id=input->arg1;
		std::list<CapPoint>::iterator itc;
		
		for(itc=get_global_agv_instance()->caps.begin();itc!=get_global_agv_instance()->caps.end();itc++){
			CapPoint act = *itc;
			if(act.id==id){
				printf("before erase\n");
				get_global_agv_instance()->caps.erase(itc);
				printf("after erase\n");
				break;
			}
		}
		
		CapPoint cap(id,pose.x,pose.y);
		get_global_agv_instance()->caps.push_back(cap);
		for(itc=get_global_agv_instance()->caps.begin();itc!=get_global_agv_instance()->caps.end();itc++){
			CapPoint act = *itc;
			printf("x=%lf,y=%lf,id=%d\n",act.x,act.y,act.id);
		}
		get_global_storage()->save_caps(get_global_agv_instance()->caps);
		
    }
	else if (func == 131)//????????????
    {
		printf("###clm:rcv msg 131,????????????\n");
		
		std::list<CapRoute>::iterator itc;
		
		for(itc=get_global_agv_instance()->routes.begin();itc!=get_global_agv_instance()->routes.end();itc++){
			CapRoute cr = *itc;
			if(cr.id==input->arg1){
				printf("before erase\n");
				get_global_agv_instance()->routes.erase(itc);
				printf("after erase\n");
				break;
			}
		}
		CapRoute cap_route;
		cap_route.id=input->arg1;
		cap_route.op_id=0xff;
		int size = input->arg2;
        int *caps = (int*)input->data_buf;
		for(int i=0;i<size;i++){
			int cap_id=caps[i];
			printf("###cap_id=%d\n",cap_id);
			for(CapPoint &point : get_global_agv_instance()->caps) {
				if(point.id==cap_id){
					CapPoint p(point.id,point.x,point.y,point.theta,point.dir,point.collision);//20210122?????????
					cap_route.cap_points.push_back(p);
					printf("id=%d,x=%lf,y=%lf\n",p.id,p.x,p.y);
				}
			}
		}
		get_global_agv_instance()->routes.push_back(cap_route);
		
		get_global_storage()->save_routes(get_global_agv_instance()->routes);
		printf("route_id=%d\n",cap_route.id);
		for(CapPoint &point : cap_route.cap_points) {
			printf("id=%d,x=%lf,y=%lf,dir=%d\n",point.id,point.x,point.y,point.dir);
		}
		printf("clm:leave msg 31,route_num=%d\n",get_global_agv_instance()->routes.size());
    }
	else if (func == 132)//????????????
    {
		printf("###clm:rcv msg 132,????????????\n");
		
		for(CapRoute &route : get_global_agv_instance()->routes) {
			if(route.id==input->arg1){
				printf("clm:route.id=%d\n",input->arg1);
				RouteChain chain;
				CapRoute r=route;
				chain.id=-1;
				chain.routes.push_back(r);
				get_global_agv_instance()->start_navigating(chain);
				return;
			}
		}
		printf("clm:leave msg 32,route_num=%d\n",get_global_agv_instance()->routes.size());
    }
	else if (func == 148)//???????????????
    {
		printf("###clm:rcv msg 1132,???????????????\n");
		
		for(RouteChain &chain : get_global_agv_instance()->chains) {
			if(chain.id==input->arg1){
				printf("clm:chain.id=%d\n",input->arg1);
				int i=0;
				for(CapRoute &route : chain.routes) {
					printf("route index=%d,id=%d,x=%lf,y=%lf,dir=%d\n",i,route.id);
					i++;
				}
				get_global_agv_instance()->start_navigating(chain);
				return;
			}
		}
		printf("clm:leave msg 32,route_num=%d\n",get_global_agv_instance()->routes.size());
    }
	else if (func == 147)//???????????????
    {
		printf("###clm:rcv msg 147,???????????????\n");
		
		std::list<RouteChain>::iterator itc;
		
		for(itc=get_global_agv_instance()->chains.begin();itc!=get_global_agv_instance()->chains.end();itc++){
			RouteChain rc = *itc;
			if(rc.id==input->arg1){
				printf("before erase\n");
				get_global_agv_instance()->chains.erase(itc);
				printf("after erase\n");
				break;
			}
		}
		RouteChain chain;
		chain.id=input->arg1;
		int size = input->arg2;
        int *routes = (int*)input->data_buf;
		for(int i=0;i<size;i++){
			int route_id=routes[i];
			printf("###route_id=%d\n",route_id);
			for(CapRoute &r : get_global_agv_instance()->routes) {
				if(r.id==route_id){
					CapRoute route=r;	//todo  test
					chain.routes.push_back(route);
					printf("route id=%d,points.size=%d\n",route_id,route.cap_points.size());
				}
			}
		}
		get_global_agv_instance()->chains.push_back(chain);
		
		get_global_storage()->save_chains(get_global_agv_instance()->chains);
		printf("chain_id=%d\n",chain.id);
		for(CapRoute &route : chain.routes) {
			printf("route id=%d\n",route.id);
		}
		printf("clm:leave msg 147,chain_num=%d\n",get_global_agv_instance()->chains.size());
    }
	else if (func == 144)//????????????
    {
		printf("###clm:rcv msg 144,????????????\n");
    }
	else if (func == 133)//????????????
    {
		printf("###clm:rcv msg 133,????????????\n");
		
		get_global_agv_instance()->stop_navigating();
    }
	else if (func == 136)//????????????
    {
		printf("###clm:rcv msg 136,????????????\n");
		
		for(CapRoute &route : get_global_agv_instance()->routes) {
			if(route.id==input->arg1){
				if(input->arg2==0){//??????
					printf("###??????%d,?????????%d\n",input->arg1,input->arg2);
					route.op_id=0;
					route.op_point.ops.clear();
					//???5???,??????,???5???,????????????,???????????????
					Operation op1,op2,op3,op4,op5;
					op1.type=OP_DELAY;
					op1.delay.sec=3;
					route.op_point.ops.push_back(op1);
					op2.type=OP_LIFT_UP;
					op2.lift.dir=1;
					route.op_point.ops.push_back(op2);
					op3.type=OP_DELAY;
					op3.delay.sec=3;
					route.op_point.ops.push_back(op3);
					op4.type=OP_RADAR;
					op4.radar.mode=1;
					route.op_point.ops.push_back(op4);
					op5.type=OP_MASK;
					op5.mask.onoff=true;
					route.op_point.ops.push_back(op5);
				}else if(input->arg2==1){//??????
					route.op_id=1;
					route.op_point.ops.clear();
					printf("###??????%d,?????????%d\n",input->arg1,input->arg2);
					//???5???,??????,???5???,???????????????
					Operation op1,op2,op3,op4;
					op1.type=OP_DELAY;
					op1.delay.sec=3;
					route.op_point.ops.push_back(op1);
					op2.type=OP_LIFT_DOWN;
					op2.lift.dir=0;
					route.op_point.ops.push_back(op2);
					op3.type=OP_DELAY;
					op3.delay.sec=3;
					route.op_point.ops.push_back(op3);
					op4.type=OP_MASK;
					op4.mask.onoff=false;
					route.op_point.ops.push_back(op4);
				}
				get_global_storage()->save_routes(get_global_agv_instance()->routes);
			}
		}
    }
	else if (func == 139)//???????????????
    {
		printf("###clm:rcv msg 139,????????????,route=%d,index=%d,dir=%d,collision=%d\n",input->arg1,input->arg2,input->data_buf[0],input->data_buf[1]);
		std::list<CapPoint>::iterator iter;
		for(CapRoute &route : get_global_agv_instance()->routes) {//???route
			if(route.id==input->arg1){
				int index=input->arg2;
				if(route.cap_points.size()>index){
					iter = route.cap_points.begin();
					advance(iter,index);
					iter->dir=input->data_buf[0];
					iter->collision=input->data_buf[1];
				}
			}else{
//				printf("###no match\n");
			}
		}
		get_global_storage()->save_routes(get_global_agv_instance()->routes);
    }
    else if (func == 142)//????????????,1??????,0??????
    {
		printf("###clm:rcv msg 142,????????????\n");
		MoveInstruction move;
		move.type = MOVE_INSTRUCTION_TYPE_LIFT;
		move.lift.dir=input->arg1;
		std::vector<char> data;
		move.to_char_array(&data);
		get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
    }
    else if (func == 146)//????????????
    {
		printf("###clm:rcv msg 146,??????????????????\n");
		MoveInstruction move;
		move.type = MOVE_INSTRUCTION_TYPE_LIFT_HEIGHT;
		move.lift.height=input->arg1;
		std::vector<char> data;
		move.to_char_array(&data);
		get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
    }
	else if (func == 135)//??????????????????
    {
		printf("###clm:rcv msg 135,????????????\n");
    	
		if(input->arg1==1){//??????
			get_global_agv_instance()->collision_switch=true;
			printf("###clm:rcv msg 135,collision=on\n");
		}else{
			get_global_agv_instance()->collision_switch=false;
			get_global_agv_instance()->collision_level=0;
			printf("###clm:rcv msg 135,collision=off\n");
			MoveInstruction move;
			move.type = MOVE_INSTRUCTION_TYPE_COLLISION;
			move.collision.level=0;
			std::vector<char> data;
			move.to_char_array(&data);
			get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
		}
    }
	else if (func == 250)//?????????
    {
		printf("###clm:rcv msg 250,?????????\n");
    	
		if(input->arg1==1){//??????
			get_global_agv_instance()->mask_switch=true;
			printf("###clm:rcv msg 250,mask_switch=on\n");
		}else{
			get_global_agv_instance()->mask_switch=false;
			printf("###clm:rcv msg 250,mask_switch=off\n");
		}
    }
	else if (func == 40)//????????????
    {
		int size = input->data_size;
		get_global_agv_instance()->map_id=input->arg1;
		memcpy(get_global_agv_instance()->map_name, input->data_buf, size);
		printf("msg 40,????????????:id=%d,name=%s\n",get_global_agv_instance()->map_id, input->data_buf);
		
        get_global_agv_instance()->start_mapping();
    }
	else if (func == 41)//??????????????????
    {
		string s=get_global_storage()->get_map_list();
		printf("msg 41,??????????????????:%s\n",s.c_str());
		int size=s.size();
		output->arg1=size;
		output->reserve_buf(size);
		memcpy(output->data_buf,s.c_str(),size);
    }
	else if(func == 42){	//????????????
		int id=input->arg1;
		string map_name=get_global_storage()->get_map_slot(id);
		printf("msg 42,????????????:id=%d,name=%s\n",id,map_name.c_str());
		FILE *fp;
		if((map_name.size()==0)||((fp=fopen(map_name.c_str(),"rb"))==NULL)){
			printf("error open file\n");
			return;
		}else{
			fseek(fp, 0, SEEK_END);
			long size = ftell(fp);
			output->reserve_buf(size);
			printf("size=%d\n",size);
			fseek(fp, 0, SEEK_SET);
			int ret;
			int read_size=0;
			while (read_size<size) {
				ret=fread(output->data_buf+read_size,1,size-read_size,fp);
				read_size+=ret;
			}
			fclose(fp);
		}
	}
	else if(func == 43){	//??????????????????
	
		int id=input->arg1;
		int map_name_len=input->arg2;
		char map_name[100];
		memset(map_name,0,100);
		memcpy(map_name,input->data_buf,map_name_len);
		int file_size= input->data_size-map_name_len;
		
		printf("msg 43:id=%d,name=%s,file_size=%d\n",id,map_name,file_size);
		FILE *fp;
		if((fp=fopen(map_name,"wb"))==NULL){
			printf("error open file\n");
			return;
		}else{
			int ret;//
			int write_len=0;
			while(write_len<file_size){
				ret=fwrite(input->data_buf+map_name_len+write_len,1,file_size-write_len,fp);
				write_len+=ret;
			}
			fclose(fp);
			get_global_storage()->modify_map_slot(id,map_name);
		}
	}
	else if(func == 44){	//??????????????????
	
		int id=input->arg1;
		printf("msg 44:id=%d\n",id);
		get_global_storage()->delete_map_slot(id);
	}
	else if(func == 45){	//??????????????????
		int id=input->arg1;
		printf("msg 45:id=%d\n",id);
		get_global_agv_instance()->default_locate_map_id=id;
		get_global_storage()->modify_default_map(id);
	}
	/*
	else if(func == 46){	//????????????2
		string map_name=get_global_storage()->get_map_slot(get_global_agv_instance()->default_locate_map_id);
		printf("msg 46:id=%d,name=%s\n",get_global_agv_instance()->default_locate_map_id,map_name.c_str());
		FILE *fp;
		char* data_buf;
		long file_len;
		if((map_name.size()==0)||((fp=fopen(map_name.c_str(),"rb"))==NULL)){
			printf("error open file\n");
			output->arg1=1;
		}else{
			fseek(fp, 0, SEEK_END);
			file_len = ftell(fp);
			data_buf = (char *)malloc(file_len);
			printf("size=%d\n",file_len);
			fseek(fp, 0, SEEK_SET);
			int ret;
			int read_size=0;
			while (read_size<file_len) {
				ret=fread(data_buf+read_size,1,file_len-read_size,fp);
				printf("read %d byte\n",ret);
				read_size+=ret;
			}
			fclose(fp);
			printf("read ok=%d\n",file_len);
			output->arg1=0;
//			get_global_agv_instance()->load_grid_map(data_buf,file_len);
//			get_global_agv_instance()->start_global_locating();
		}
	}
	else if (func == 50)//??????route??????
    {
		printf("msg 50\n");
		string s=get_global_storage()->get_route_list();
		printf("%s\n",s.c_str());
		int size=s.size();
		output->reserve_buf(size);
		memcpy(output->data_buf,s.c_str(),size);
    }
	else if (func == 51)//??????route
	{
		printf("msg 51\n");
		get_global_storage()->modify_route_slot(input->arg1,input->data_buf);
    }
	else if (func == 52)//??????cap
    {
		printf("msg 52,data=%s,len=%d\n",input->data_buf,input->data_size);
		//??????
		char* buf=(char*)malloc(input->data_size+1);
		memset(buf,0,input->data_size+1);
		memcpy(buf,input->data_buf,input->data_size);
		
		get_global_storage()->modify_cap_slot(input->arg1,input->arg2,buf);
    }
	else if (func == 53)//??????route
    {
		printf("msg 53\n");
		get_global_storage()->delete_route_slot(input->arg1);
    }
	else if (func == 54)//??????cap
    {
		printf("msg 54\n");
		get_global_storage()->delete_route_cap(input->arg1,input->arg2);
    }
    else if (func == 55)//??????cap
    {
		printf("###clm:rcv msg 55\n");
		Position pose = get_global_agv_instance()->expolate_current_position();
		get_global_storage()->modify_cap_slot(input->arg1,input->arg2,pose.x,pose.y,pose.theta);
    }
    else if (func == 56)//????????????
    {
		printf("###clm:rcv msg 56\n");
		
		CapRoute cap_route;
		cap_route.id=input->arg1;
		
		for(int i=0;i<CAP_SLOT_NUM;i++){
			if(get_global_agv_instance()->r[input->arg1].cap[i].inuse==1){
				CapPoint capn(i,get_global_agv_instance()->r[input->arg1].cap[i].x,get_global_agv_instance()->r[input->arg1].cap[i].y);
				
				cap_route.cap_points.push_back(capn);
			}
		}
		
		int index=0;
		for(CapPoint &point : cap_route.cap_points) {
			printf("index=%d,id=%d,x=%lf,y=%lf\n",index,point.id,point.x,point.y);
			index++;
		}
		get_global_agv_instance()->start_navigating(&cap_route);
    }*/
}

void error_handling(char *message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}

void handle_socket(void *param)
{
    int clnt_sock = *((int*)param);
    delete param;
    int extra_len = 0, rec_len = 0, total_len = 0, snd_len = 0;
    int ret = 0;
    char buf_head[HEAD_SIZE];

    ret = read(clnt_sock, buf_head, HEAD_SIZE);
    if (ret != HEAD_SIZE)
    {
        printf("[LOG] Read package head error \n");
        close(clnt_sock);
        return;
    }
    TCPDataPackage *recv = new TCPDataPackage(buf_head, &ret);
    if (ret != 0)
    {
        printf("[LOG] Package check failed \n");
        close(clnt_sock);
        delete recv;
        return;
    }
    extra_len = recv->data_size;
    rec_len = 0;
    if (recv->data_size > 0)
    {
        while (rec_len < extra_len)
        {
            ret = read(clnt_sock, recv->data_buf + rec_len, extra_len - rec_len);
            if (ret < 0)
            {
                break;
            }
            rec_len += ret;
        }
    }
    if (rec_len < extra_len)
    {
        printf("[LOG] Read extra data failed \n");
        close(clnt_sock);
        delete recv;
        return;
    }
	
    TCPDataPackage *respond = new TCPDataPackage(recv->function);
    handle_package(recv, respond);

    respond->to_bytes(buf_head);
    write(clnt_sock, buf_head, HEAD_SIZE);
    extra_len = respond->data_size;
    snd_len = 0;
    if (extra_len > 0)
    {
        while (snd_len < extra_len)
        {
            ret = write(clnt_sock, respond->data_buf + snd_len, extra_len - snd_len);
            if (ret < 0)
            {
                break;
            }
            snd_len += ret;
        }
    }
    if (snd_len < extra_len)
    {
        printf("[LOG] Send data failed \n");
        close(clnt_sock);
    }

    shutdown(clnt_sock, 2);
    usleep(200000);
    close(clnt_sock);

    delete recv;
    delete respond;
}

void running_network()
{
    //Todo
    //??????????????????????????????????????????
    //?????????????????????main?????????????????????????????????????????????
    //?????????????????????????????????????????????

    int serv_sock;
    int clnt_sock;

    struct sockaddr_in serv_addr;
    struct sockaddr_in clnt_addr;
    socklen_t clnt_addr_size;

    threadpool thread_pool = thpool_init(get_configs()->get_int("network", "num_threads", nullptr));

    serv_sock = socket(PF_INET, SOCK_STREAM, 0);
    if (serv_sock == -1)
    {
        error_handling("socket() error");
    }
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(local_port);
    if (bind(serv_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
    {
        error_handling("bind() error");
    }
    if (listen(serv_sock, 5) == -1)
    {
        error_handling("listen() error");
    }

    clnt_addr_size = sizeof(clnt_addr);
    printf("START LISTENING AT PORT %d\n", local_port);

    while (1)
    {
        clnt_sock = accept(serv_sock, (struct sockaddr *)&clnt_addr, &clnt_addr_size);
        if (clnt_sock == -1)
        {
            printf("[LOG] Accept error :%s\n", strerror(errno));
            continue;
        }
        int *p_socket = new int(clnt_sock);
        thpool_add_work(thread_pool, handle_socket, (void*)p_socket);
    }

    thpool_destroy(thread_pool);
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Usage:");
        printf("  ./agv_main xxx/configuration_file.txt\n");
        return -1;
    }
    load_config(argv[1]);
    local_port = get_configs()->get_int("network", "local_port", nullptr);
    if(argc > 2) {
        local_port = atoi(argv[2]);
    }

    init_global_agv_instance();
	

	get_global_agv_instance()->default_locate_map_id=get_global_storage()->init_map_db();
	
	
	get_global_storage()->init_db();
	
	get_global_agv_instance()->exec_route=0xffff;
	/*
	get_global_agv_instance()->caps.push_back(CapPoint(0,0.0,1.0,180.0,1,1));
	get_global_agv_instance()->caps.push_back(CapPoint(1,1.0,1.0,180.0,1,1));
	get_global_agv_instance()->caps.push_back(CapPoint(12,12.0,1.0,180.0,1,1));
	get_global_agv_instance()->caps.push_back(CapPoint(15,12.0,1.0,180.0,1,1));
	get_global_agv_instance()->caps.push_back(CapPoint(17,12.0,1.0,180.0,1,1));
	
	
	for(int i=0;i<5;i++){
		CapRoute cap_route;
		cap_route.id=i;
		cap_route.op_id=i%2;
		cap_route.cap_points.push_back(CapPoint(0,0.0,1.0,180.0,1,1));
		cap_route.cap_points.push_back(CapPoint(1,1.0,1.0,180.0,1,1));
		cap_route.cap_points.push_back(CapPoint(12,12.0,1.0,180.0,1,1));
		cap_route.cap_points.push_back(CapPoint(15,12.0,1.0,180.0,1,1));
		cap_route.cap_points.push_back(CapPoint(17,12.0,1.0,180.0,1,1));
		get_global_agv_instance()->routes.push_back(cap_route);
	}
	get_global_storage()->save_caps(get_global_agv_instance()->caps);
	get_global_storage()->save_routes(get_global_agv_instance()->routes);
	
	
	get_global_agv_instance()->routes.clear();
	get_global_storage()->init_db();
	std::list<CapPoint>::iterator iter;
	for(CapPoint &cap : get_global_agv_instance()->caps) {//???route
		printf("id=%d,x=%lf\n",cap.id,cap.x);
	}
	*/
	
    running_network();

    delete get_global_agv_instance();
    return 0;
};

/*
|-	4b		-|-1b-|--	3b	--|--			8b				--|--	4b		--|--	4b		--|	
|-	?????????	-|??????|--?????????	--|--			??????			--|-??????????????????--|--	??????	--|			
0	1	2	3	4	5	6	7	8	9	10	11	12	13	14	15	16	17	18	19	20	21	22	23
xx	xx	xx	xx	03	00	00	00	00	00	00	00	00	00	00	00	00	00	03	e8	00	00	00	00	??????1000????????????



1??????????????????4Byte?????????int???????????????????????????????????????????????????????????????????????????????????????????????????????????????0 

2???????????????????????????0???????????????????????????1???short?????????????????????????????????2???6???unsigned char????????????????????????????????????????????????????????????4???double?????????????????????x???y?????????????????

3???????????????????????????0??????????????????????????????0??????????????????????????????????????????

7???????????????????????????0??????????????????????????????0??????????????????????????????

17??????????????????????????????double????????????????????????????????????????????????0??????1??????
19??????????????????18

18????????????????????????0???????????????????????????0??????1??????
21??????????????????18

23????????????????????????0??????????????????double?????????????????????????????????0??????1??????
24??????????????????18
*/
