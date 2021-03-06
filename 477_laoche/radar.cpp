
#include <stdlib.h>
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

#include "agv.h"
#include "radar.h"
#include "common/time.h"
#include "common/configuration.h"

RadarModule::RadarModule()
{
	pthread_mutex_init(&map_mutex, nullptr);
	pthread_create(&radar_thread, NULL, thread_function, this);
};

RadarModule::~RadarModule()
{
	thread_running = false;
	close_radar();
	pthread_mutex_destroy(&map_mutex);
	pthread_join(radar_thread, NULL);
};

#define PI 3.1415926
#define MAXLINE 40960
void RadarModule::make_cmd_frame(enum RadarModule::CMD cmd)
{
	char const *tmp;
	memset(send_buf, 0, 1024);
	send_buf[0] = 0x02;
	switch (cmd)
	{
	case SINGLE_READ:
		tmp = cmd_single_read;
		break;
	case CONTINUE_READ_START:
		tmp = cmd_continue_read_start;
		break;
	case CONTINUE_READ_STOP:
		tmp = cmd_continue_read_stop;
		break;
	default:
		break;
	}
	memcpy(send_buf + 1, tmp, strlen(tmp));
	send_buf[strlen(tmp) + 1] = 0x03;
};

void RadarModule::make_cmd_frame2(enum RadarModule::CMD cmd)
{
	memset(send_buf, 0, 1024);
	send_buf[0] = 0x02;
	send_buf[1] = 0x73;
	send_buf[2] = 0x52;
	send_buf[3] = 0x4e;
	send_buf[4] = 0x20;
	send_buf[5] = 0x4c;
	send_buf[6] = 0x4d;
	send_buf[7] = 0x44;
	send_buf[8] = 0x73;
	send_buf[9] = 0x63;
	send_buf[10] = 0x61;
	send_buf[11] = 0x6e;
	send_buf[12] = 0x64;
	send_buf[13] = 0x61;
	send_buf[14] = 0x74;
	send_buf[15] = 0x61;
	send_buf[16] = 0x03;
};

void *RadarModule::thread_function(void *param)
{

	RadarModule *ptr = (RadarModule *)param;

	//这里是线程参数
	//初始化并启动IMU
	//在这里用 ptr->xxx 的形式访问你定义的中间变量，就像下面while那一行里那样
	//------------------

	int *range, *rssi;
	int lazer_cnt = 0;
	char *temp1, *temp2;
	char *cmdtype, *cmd, *channel_content;
	int version, device_number, serial_number, device_status_l, device_status_h;
	int telegram_counter, scan_counter, start_time, xmission_time, dinput_l, dinput_h, doutput_l, doutput_h, rsvd;
	int scan_freq, measure_freq;
	int encoder_num, encoder_pos, encoder_speed;
	int channel_num, scale_fac, start_point, start_angle, angle_step, data_num;
	int i, j;

	double offset_x = get_configs()->get_float("radar", "radar_position_x", nullptr);
	double offset_y = get_configs()->get_float("radar", "radar_position_y", nullptr);
	int from_index = get_configs()->get_int("radar", "first_point_index", nullptr);
	int to_index = get_configs()->get_int("radar", "last_point_index", nullptr);
	int point_number = get_configs()->get_int("radar", "point_number", nullptr);
	
	//level1是小圈,红色报警线
	double l1x=get_configs()->get_float("radar", "collision_distance_level1x", nullptr);
	double l1y=get_configs()->get_float("radar", "collision_distance_level1y", nullptr);
	double l2x=get_configs()->get_float("radar", "collision_distance_level2x", nullptr);
	double l2y=get_configs()->get_float("radar", "collision_distance_level2y", nullptr);
	double l3x=get_configs()->get_float("radar", "collision_distance_level3x", nullptr);
	double l3y=get_configs()->get_float("radar", "collision_distance_level3y", nullptr);
	int thsh = get_configs()->get_int("radar", "collision_threshold", nullptr);
	
	int n, rec_len, total_len;
	char recvline[4096], sendline[4096];
	char buf[MAXLINE];
	struct sockaddr_in servaddr;
	char *pbuf;
	if ((ptr->sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("create socket error: %s\n", strerror(errno));
		return nullptr;
	}

	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(ptr->port);
	if (inet_pton(AF_INET, ptr->host_name, &servaddr.sin_addr) <= 0)
	{
		printf("inet_pton error for %s\n", ptr->host_name);
		return nullptr;
	}

	if (connect(ptr->sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
	{
		printf("connect error: %s(errno: %d) \n", strerror(errno), errno);
		return nullptr;
	}

	//	make_cmd_frame(SINGLE_READ);
	ptr->make_cmd_frame(CONTINUE_READ_START);

	printf("radar :send msg to server: %s,len=%d \n", ptr->send_buf, (int)strlen(ptr->send_buf));

	if (send(ptr->sockfd, ptr->send_buf, strlen(ptr->send_buf), 0) < 0)
	{
		printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
		return nullptr;
	}
	int mtu_f = 0;

	printf("radar start\n");

	while (ptr->thread_running)
	{
		//循环读取数据
		mtu_f = 0;
		rec_len = 0;
		total_len = 0;
		while (1)
		{
			if ((rec_len = recv(ptr->sockfd, buf + total_len, MAXLINE, 0)) == -1)
			{
				perror("recv error");
				return nullptr;
			}
			total_len += rec_len;
			if (buf[total_len - 1] == 0x03)
			{
				break;
			}
			else
			{
				mtu_f = 1;
			}
		}
		rec_len = total_len;
		buf[rec_len] = '\0';
		//		printf("#rcv lazer frame. time=%ld\n",get_current_time());
		rec_len = total_len;

		buf[rec_len] = '\0';
		if (buf[0] == 0x02)
		{
			//printf("Received : %s \n\n",buf);
			cmdtype = strtok(buf + 1, " ");
			if (strcmp(cmdtype, "sSN") == 0)
			{
				cmd = strtok(NULL, " ");
				sscanf(strtok(NULL, " "), "%x", &version);
				sscanf(strtok(NULL, " "), "%x", &device_number);
				sscanf(strtok(NULL, " "), "%x", &serial_number);
				sscanf(strtok(NULL, " "), "%x", &device_status_l);
				sscanf(strtok(NULL, " "), "%x", &device_status_h);
				sscanf(strtok(NULL, " "), "%x", &telegram_counter);
				sscanf(strtok(NULL, " "), "%x", &scan_counter);
				sscanf(strtok(NULL, " "), "%x", &start_time);
				sscanf(strtok(NULL, " "), "%x", &xmission_time);

				sscanf(strtok(NULL, " "), "%x", &dinput_l);
				sscanf(strtok(NULL, " "), "%x", &dinput_h);
				sscanf(strtok(NULL, " "), "%x", &doutput_l);
				sscanf(strtok(NULL, " "), "%x", &doutput_h);
				sscanf(strtok(NULL, " "), "%x", &rsvd);
				sscanf(strtok(NULL, " "), "%x", &scan_freq);
				sscanf(strtok(NULL, " "), "%x", &measure_freq);
				sscanf(strtok(NULL, " "), "%x", &encoder_num);

				if (encoder_num != 0)
				{
					sscanf(strtok(NULL, " "), "%x", &encoder_pos);
					sscanf(strtok(NULL, " "), "%x", &encoder_speed);
				}
				sscanf(strtok(NULL, " "), "%x", &channel_num);
				channel_content = strtok(NULL, " ");
				sscanf(strtok(NULL, " "), "%x", &scale_fac);
				sscanf(strtok(NULL, " "), "%x", &start_point);
				sscanf(strtok(NULL, " "), "%x", &start_angle);
				sscanf(strtok(NULL, " "), "%x", &angle_step);
				sscanf(strtok(NULL, " "), "%x", &data_num);

				range = (int *)malloc(sizeof(int) * data_num);
				rssi = (int *)malloc(sizeof(int) * data_num);
				for (i = 0; i < data_num; i++)
				{
					sscanf(strtok(NULL, " "), "%x", range +  i);
					//sscanf(strtok(NULL, " "), "%x", range + i);
				}

				sscanf(strtok(NULL, " "), "%x", &channel_num);
				channel_content = strtok(NULL, " ");
				sscanf(strtok(NULL, " "), "%x", &scale_fac);
				sscanf(strtok(NULL, " "), "%x", &start_point);
				sscanf(strtok(NULL, " "), "%x", &start_angle);
				sscanf(strtok(NULL, " "), "%x", &angle_step);
				sscanf(strtok(NULL, " "), "%x", &data_num);
				//printf("\n ");
				for (i = 0; i < data_num; i++)
				{
					sscanf(strtok(NULL, " "), "%x", rssi +  i);
					//sscanf(strtok(NULL, " "), "%x", rssi +  i);
					if (rssi[i] > 100)
						rssi[i] = 100;
					//printf("%d ",rssi[i]);
				}
				//printf("\n ");
				/*
				^  x
				|	
				|	
				*/
				lazer_cnt++;
				if ((lazer_cnt % 5) == 0)
				{
					long timestamp_us = get_current_time_us();
					int data_size = point_number; //点的数量，用于预分配空间
					PointCloudData data(timestamp_us, data_size);
					// printf("#rcv lazer frame. time=%ld\n",timestamp_us);
					
					get_global_agv_instance()->colli_cnt1=0;
					get_global_agv_instance()->colli_cnt2=0;
					get_global_agv_instance()->colli_cnt3=0;
					int cnt=0;
					for (int i = from_index; i <= to_index; i++)//60-480
					{
						double x = (0. - sin((45.0 - i / 2.) / 180. * PI) * range[i]) / 1000. ;
						double y = (0. + cos((45.0 - i / 2.) / 180. * PI) * range[i]) / 1000. ;
						bool leg=false;
						
						if(get_global_agv_instance()->mask_switch&&(get_global_agv_instance()->masks.size()>0)){
							for (auto iter = get_global_agv_instance()->masks.begin();; iter++){
								double dis=pow(x-iter->x,2)+pow(y-iter->y,2);
								if(iter==get_global_agv_instance()->masks.end()){
									break;
								}
								if (dis<pow(get_global_agv_instance()->mask_radius,2)){
									leg=true;
									break;
								}
							}
							if(leg==false){
								data.add_point(x, y, rssi[i] / 100.0);
								cnt++;
							}
						}else{
							data.add_point(x, y, rssi[i] / 100.0);
							cnt++;
						}
						
						
						if(get_global_agv_instance()->collision_switch){//避障
							if((x>0.)&&(x<l1x)&&(y<l1y)&&(y>0-l1y)){
								get_global_agv_instance()->colli_cnt1++;
							}
							if((x>0.)&&(x<l2x)&&(y<l2y)&&(y>0-l2y)){
								get_global_agv_instance()->colli_cnt2++;
							}
							if((x>0.)&&(x<l3x)&&(y<l3y)&&(y>0-l3y)){
								get_global_agv_instance()->colli_cnt3++;
							}
						}
					}
					
//					printf("####rcv lazer frame. report %d point,masks.size=%d\n",cnt,get_global_agv_instance()->masks.size());
					if(get_global_agv_instance()->collision_switch){
						int new_level;
						if(get_global_agv_instance()->colli_cnt1>thsh){
							new_level=1;
						}else if(get_global_agv_instance()->colli_cnt2>thsh){
							new_level=2;
						}else if(get_global_agv_instance()->colli_cnt3>thsh){
							new_level=3;
						}else{
							new_level=0;
						}
						//变更避障等级,则下发给小童
						if(get_global_agv_instance()->collision_level!=new_level){
							ptr->send_collision_msg(new_level);
							get_global_agv_instance()->collision_level=new_level;
						}
						
/*						printf("clm######collision=%d,%d,%d,level=%d\n",
						get_global_agv_instance()->colli_cnt1,
						get_global_agv_instance()->colli_cnt2,
						get_global_agv_instance()->colli_cnt3,
						get_global_agv_instance()->collision_level);*/
					}
					
					pthread_mutex_lock(&ptr->map_mutex);
					if (ptr->p_msg_queue != nullptr)
					{
						std::vector<char> tmp;
						data.to_char_array(&tmp);
						ptr->p_msg_queue->send(ptr->channel, tmp.data(), tmp.size());
					}
					pthread_mutex_unlock(&ptr->map_mutex);
				}
				free(range);
				free(rssi);
			}
		}
		else
		{
			printf("radar wrong frame!\n");
		}
	}
	printf("radar thread quit!\n");
	return nullptr;
};

void RadarModule::send_collision_msg(int level){
	MoveInstruction move;
	move.type = MOVE_INSTRUCTION_TYPE_COLLISION;
	move.collision.level=level;
	std::vector<char> data;
	move.to_char_array(&data);
	get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
}

void RadarModule::close_radar()
{
	//在这里关闭雷达
	//在这里直接用变量名就可以访问你定义的中间变量
	//如直接调用 thread_running
	//------------------
	close(sockfd);
};

void RadarModule::set_target(MsgQueue *target, int channel)
{
	pthread_mutex_lock(&map_mutex);
	p_msg_queue = target;
	this->channel = channel;
	pthread_mutex_unlock(&map_mutex);
};
