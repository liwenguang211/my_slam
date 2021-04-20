#include <stdio.h>
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

#include "device.h"
#include "common/time.h"
#include "common/configuration.h"

#define PI (3.141592653589793)



FILE *fp;
FILE *fp_1;
FILE *fp_original;
DevicesModule::DevicesModule(MsgQueue *queue) : p_msg_queue(queue)
{
    pthread_mutex_init(&mutex, nullptr);
    pthread_create(&odom_imu_thread, NULL, thread_function_odom, this);
    pthread_create(&nav_thread, NULL, thread_function_nav, this);

    char dev_name[200];
    get_configs()->get_string("device", "move_dev_name", dev_name, nullptr);
    move_fd = open_tty(dev_name, get_configs()->get_int("device", "move_baudrate", nullptr), 0);

    if (queue != nullptr)
    {
        queue->add_listener(CHANNEL_MOVE, this);
    }
	
	fp=fopen("odom_imu_angle.txt","wb");
    fp_1=fopen("imu.txt","w");
    fp_original=fopen("imu_original.txt","w");
}

DevicesModule::~DevicesModule()
{

    thread_running = false;
    pthread_join(odom_imu_thread, NULL);
    pthread_join(nav_thread, NULL);

    if (odom_imu_fd > 0)
    {
        close(odom_imu_fd);
    }
    if (nav_fd > 0)
    {
        close(nav_fd);
    }
    if (move_fd > 0)
    {
        close(move_fd);
    }
    pthread_mutex_destroy(&mutex);

    if (p_msg_queue != nullptr)
    {
        p_msg_queue->remove_listener(CHANNEL_MOVE, this);
    }
}

void DevicesModule::recieve_message(const int channel, char *buf, const int size)
{
    if (channel != CHANNEL_MOVE)
    {
        return;
    }
    MoveInstruction ins;
    ins.from_char_array(buf, size);
    printf("Get move message , length %d, type %d\n", size, ins.type);
    if (ins.type == MOVE_INSTRUCTION_TYPE_SPEED)
    {
        //handle_movement_message(ins.speed.speed, ins.speed.turn_left);
    }
    else if (ins.type == MOVE_INSTRUCTION_TYPE_TARGET)
    {
        //handle_nav_message(ins.target.start_x, ins.target.start_y, ins.target.start_theta,
	//	ins.target.x, ins.target.y, ins.target.theta,ins.target.dir);
    }
    else if (ins.type == MOVE_INSTRUCTION_TYPE_TURN)
    {
        //handle_turn_message(ins.turn.dir, ins.turn.angle);
    }
    else if (ins.type == MOVE_INSTRUCTION_TYPE_COLLISION)
    {
        //handle_collision_message(ins.collision.level);
    }
    else if (ins.type == MOVE_INSTRUCTION_TYPE_LIFT)
    {
        //handle_lift_message(ins.lift.dir);
    }
    else if (ins.type == MOVE_INSTRUCTION_TYPE_LIFT_HEIGHT)
    {
        //handle_lift_height_message(ins.lift.height);
    }
}

void *DevicesModule::thread_function_odom(void *param)
{
    DevicesModule *ptr = (DevicesModule *)param;

    int byte_index = 0, len = 0, new_frame = 0;
    char in[100];
    char frame[100];
    long timestamp = -1;

    key_t key = ftok("/", 'c');
    int msqid = msgget(key, IPC_CREAT | 0666);

    char dev_name[200];
    get_configs()->get_string("device", "odom_dev_name", dev_name, nullptr);
    ptr->odom_imu_fd = ptr->open_tty(dev_name, get_configs()->get_int("device", "odom_baudrate", nullptr), 0);

    while (ptr->thread_running)
    {
        int ret = read(ptr->odom_imu_fd, in, 8);
        timestamp = -1;

        if (ret > 0)
        {
            for (int i = 0; i < ret; i++)
            {
                if (new_frame == 0)
                {
                    if (in[i] == 0x02)
                    { //收到H
                        byte_index = 0;
                        len = 0;
                        new_frame = 1;
                    }
                }
                else if (new_frame == 1)
                {
                    //					printf("@@@@@new odom frame\n");
                    frame[byte_index] = in[i];
                    byte_index++;
                    if (byte_index == 1)
                        len = in[i];
                    if (byte_index == len)
                    {
                        timestamp = ptr->handle_frame_odom_imu(frame, len, msqid);
                        memset(frame, 0, 100);
                        new_frame = 0;
                    }
                }
            }
        }
    }

    return nullptr;
}
//
void *DevicesModule::thread_function_nav(void *param)
{
    DevicesModule *ptr = (DevicesModule *)param;

    char dev_name[200];
    get_configs()->get_string("device", "nav_dev_name", dev_name, nullptr);
    ptr->nav_fd = ptr->open_tty(dev_name, get_configs()->get_int("device", "nav_baudrate", nullptr), 0);

    int byte_index = 0, len = 0, new_frame = 0;
    char in[100];
    char frame[100];
    long timestamp = -1;

    while (ptr->thread_running)
    {
        int ret = read(ptr->nav_fd, in, 9);
        timestamp = -1;

        if (ret > 0)
        {
            for (int i = 0; i < ret; i++)
            {
                if (new_frame == 0)
                {
                    if (in[i] = 0xa0)
                    { //wheel协议
                        byte_index = 0;
                        len = 0;
                        new_frame = 2;
//                        printf("###new_frame==0\n");
                    }
                }
                else if (new_frame == 2)
                { //wheel协议第二个起始位
                    if (in[i] = 0xb0)
                    {
                        new_frame = 3;
                    }
                }
                else if (new_frame == 3)
                {
                    frame[byte_index] = in[i];
                    if (byte_index == 2)
                    {
                        len = frame[byte_index] + frame[byte_index - 1] * 0xff;
                    }
                    if (byte_index == (len - 3))
                    {
                        timestamp = ptr->handle_frame_odom3(frame, len);
                        memset(frame, 0, 100);
                        new_frame = 0;
                    }
                    byte_index++;
                }
            }
        }
	//usleep(5000);
    }
    return nullptr;
}

long DevicesModule::handle_frame_odom3(char *p, int len)
{
    char hex2str[300];

    long timestamp = get_current_time_us();
    int type = -1; //左右轮
    long data = 0; //传感器数据

    memset(hex2str, 0, 300);
    sprintf(hex2str, "0a 0b ");
    for (int j = 0; j < len; j++){
        sprintf(hex2str + 6 + j * 3, "%02x ", p[j]);
    }
    int cmd = p[3] * 0x100 + p[4];
	switch(cmd){
		case 0xc101://上报到达
		case 0xc107://上报导航转向完成/顶升转向完成
		case 0xc103://上报顶升上升完成
		case 0xc104://上报顶升下降完成
			if (p_msg_queue != nullptr){
				p_msg_queue->send(CHANNEL_ARRIVE, (char *)(&cmd), sizeof(int));
			}
			break;
		default:
			break;
	}
	
	/*
    if (cmd == 0xc101){
        //上报到达状态
        if (p_msg_queue != nullptr){
            p_msg_queue->send(CHANNEL_ARRIVE, (char *)(&cmd), sizeof(int));
        }
    }
    else if (cmd == 0xc107){
        //上报转向完成状态
        if (p_msg_queue != nullptr){
            p_msg_queue->send(CHANNEL_ARRIVE, (char *)(&cmd), sizeof(int));
        }
    }
    else if (cmd == 0xc103){
        //上报顶升上升完成状态
        if (p_msg_queue != nullptr){
            p_msg_queue->send(CHANNEL_ARRIVE, (char *)(&cmd), sizeof(int));
        }
    }
    else if (cmd == 0xc104){
        //上报顶升下降完成状态
        if (p_msg_queue != nullptr){
            p_msg_queue->send(CHANNEL_ARRIVE, (char *)(&cmd), sizeof(int));
        }
    }*/
    return timestamp;
}
int DevicesModule::toBCD(int x){
	return ((x%256)/16)*10+(x%16);
}

static long std_stamp=0;
static int left_odom_counter=0;
static long stamp_sensor_ms=0;
static long former_stamp_sensor_ms=0;
static int lcount = 0, rcount = 0;
long DevicesModule::handle_frame_odom_imu(char *p, int len, int msqid)
{
    char hex2str[300];
    long timestamp = get_current_time_us();
    int type = -1; //左右轮
    long data = 0; //传感器数据

    memset(hex2str, 0, 300);
    sprintf(hex2str, "02 ");
    for (int j = 0; j < len; j++)
    {
        sprintf(hex2str + 3 + j * 3, "%02x ", p[j]);
    }
//
//    time_ms = timestamp / 10000;
	time_ms = timestamp / 10000;
    int cmd = p[1];
    long odomd;
    if (cmd == 0xf1)
    { //左
        //		lcount++;
        //		if(lcount%2){
        //if (time_ms >= l_send_time)
        {
            odomd = p[2] * 256 * 256 * 256 + p[3] * 256 * 256 + p[4] * 256 + p[5];
			/*
			stamp_sensor_ms=p[6] * 256 * 256 * 256 + p[7] * 256 * 256 + p[8] * 256 + p[9];
			
			if(left_odom_counter>=0){
				left_odom_counter++;
				if(left_odom_counter==20){
					left_odom_counter=-1;
					std_stamp=get_current_time_us()-stamp_sensor_ms*1000;
					printf("odom rcv:%s\n",hex2str);
					printf("###save stamp,stamp_sensor_ms=%ld,std_stamp=%ld,current_time=%ld\n",stamp_sensor_ms,std_stamp,timestamp);
				}
			}else if(left_odom_counter==-1){
				timestamp=stamp_sensor_ms*1000+std_stamp;
				
//				printf("odom rcv:%s\n",hex2str);
			}*/
            type = WHEEL_DATA_TYPE_LEFT;
            data = 0 - odomd;
		//printf("111111111111111111111111111left odom delta_time =%ld\n",timestamp-l_send_time);
            //l_send_time = time_ms + 2;
		l_send_time = timestamp;
		//	fprintf(fp,"%ld,%s,%d\n",timestamp,"-1",data);
            send_wheel_data(WheelSensorData(timestamp, type, data));
        }
        //		}
    }
    else if (cmd == 0xf2)
    {
        //		rcount++;
        //		if(rcount%2){
        //if (time_ms >= r_send_time)
        {
            odomd = p[2] * 256 * 256 * 256 + p[3] * 256 * 256 + p[4] * 256 + p[5];
			/*
			stamp_sensor_ms=p[6] * 256 * 256 * 256 + p[7] * 256 * 256 + p[8] * 256 + p[9];
			
			if(left_odom_counter==-1){
				timestamp=stamp_sensor_ms*1000+std_stamp;

//				printf("odom rcv:%s\n",hex2str);
			}
			*/
            type = WHEEL_DATA_TYPE_RIGHT;
            data = odomd;
		//printf("22222222222222222222222222222right odom delta_time =%ld\n",timestamp-r_send_time);
		r_send_time = timestamp;
            //r_send_time = time_ms + 2;
		//	fprintf(fp,"%ld,%s,%d\n",timestamp,"0",data);
            send_wheel_data(WheelSensorData(timestamp, type, data));
        }
        //		}
    }
    else if (cmd == 0xf3)
    {
        long timestamp_us = get_current_time_us();
/*		//modbus协议,输出格式1-
        double angular_speed = (p[2] + p[3] * 256 + p[4] * 256 * 256 + p[5] * 256 * 256 * 256 - 150000) * 0.01;
        double acceleration = (p[6] + p[7] * 256 + p[8] * 256 * 256 + p[9] * 256 * 256 * 256 - 20000) * 0.001;
        double angle = (p[10] + p[11] * 256 + p[12] * 256 * 256 + p[13] * 256 * 256 * 256 - 18000) * 0.01;
        double acc_x = acceleration * accel_g;
        double acc_y = 0.;
        double vel_z_axis = 0. - angular_speed / 180. * PI;
		printf("#####acc_x=%lf,vel_z=%lf\n",acc_x,vel_z_axis);
		
		*/
		//改为瑞芬协议,输出格式3
		
		double z_angular_velocity=(float)((p[2]&0x10)==0x10?
		(0-toBCD(p[2]&0x0f)*10000-toBCD(p[3])*100-toBCD(p[4])):
		(toBCD(p[2]&0x0f)*10000+toBCD(p[3])*100+toBCD(p[4])))/100;
		
		//m/s2
		double x_linear_accel=(float)((p[5]&0x10)==0x00?//反装
		(0-toBCD(p[5]&0x0f)*10000-toBCD(p[6])*100-toBCD(p[7])):
		(toBCD(p[5]&0x0f)*10000+toBCD(p[6])*100+toBCD(p[7])))/1000;
		
		double y_linear_accel=(float)((p[8]&0x10)==0x10?	
		(0-toBCD(p[8]&0x0f)*10000-toBCD(p[9])*100-toBCD(p[10])):
		(toBCD(p[8]&0x0f)*10000+toBCD(p[9])*100+toBCD(p[10])))/100;
		
		double z_angle=(float)((p[11]&0x10)==0x10?
		(0-toBCD(p[11]&0x0f)*10000-toBCD(p[12])*100-toBCD(p[13])):
		(toBCD(p[11]&0x0f)*10000+toBCD(p[12])*100+toBCD(p[13])))/100;
	/*
		stamp_sensor_ms=p[14] * 256 * 256 * 256 + p[15] * 256 * 256 + p[16] * 256 + p[17];
		
		if(left_odom_counter==-1){
			if(former_stamp_sensor_ms>stamp_sensor_ms){
				std_stamp+=0x100000000*1000;
				printf("###########################\n");
				printf("###########################\n");
				printf("###########################\n");
				printf("f=%ld,n=%ld\n",former_stamp_sensor_ms,stamp_sensor_ms);
				printf("###########################\n");
				printf("###########################\n");
				printf("###########################\n");
			}
			
			
			timestamp_us=stamp_sensor_ms*1000+std_stamp;
			former_stamp_sensor_ms=stamp_sensor_ms;
//			printf("odom rcv:%s\n",hex2str);
//			printf("###save stamp,stamp_sensor_ms=%ld,std_stamp=%ld,current_time=%ld\n",stamp_sensor_ms,std_stamp,timestamp_us);
		}
		*/
 //   	printf("odom rcv:%s\n",hex2str);
        double acc_x = x_linear_accel * accel_g;
        double acc_y = 0;
        double vel_z_axis = 0. - z_angular_velocity / 180. * PI;
	cnt_imu++;
	if(cnt_imu>200)
	{
	printf("11111111111imu data received====%f\n",z_angle);
	cnt_imu = 0;
	}
	z_angle = z_angle / 180. * PI;
//		printf("#####acc_x=%lf,vel_z=%lf\n",acc_x,vel_z_axis);
        //四个参数分别是时间戳，x加速度，y加速度，z轴角速度
		fprintf(fp,"%ld,%s,%.2f,%f\n",timestamp_us,"1",z_angle,(z_angle-last_imu_angle)*1000000);///////////生成文件
	last_imu_angle = z_angle;
        //fprintf(fp_1,"%ld,%s,%.2f,%.4f\n",timestamp_us,"1",z_angle,z_angular_velocity);//ahglkghadfhgakdfghadjkfghadlkfhgadklfjhglaekfhge
        //fprintf(fp_original,"%s\n",p);
        ImuData2D data(timestamp_us, acc_x, acc_y, vel_z_axis);
	data.angle_z=z_angle;
        if (p_msg_queue != nullptr)
        {
            std::vector<char> tmp;
            data.to_char_array(&tmp);
            p_msg_queue->send(CHANNEL_IMU, tmp.data(), tmp.size());
        }
    }
    else if (cmd == 0xf5)
    {
		int bufi[5];
		bufi[0]=p[5] * 0x100*0x100*0x100 + p[4]*0x100*0x100+p[3] * 0x100 + p[2];
		bufi[1]=p[7] * 0x100 + p[6];
		bufi[2]=p[8];
		bufi[3]=p[12] * 0x100*0x100*0x100 + p[11]*0x100*0x100+p[10] * 0x100 + p[9];
		bufi[4]=p[16] * 0x100*0x100*0x100 + p[15]*0x100*0x100+p[14] * 0x100 + p[13];
//		printf("recv %s,bat=%d,p8=%d\n",hex2str,bufi[2],p[8]);
		if (p_msg_queue != nullptr){
			p_msg_queue->send(CHANNEL_STATE, (char *)bufi, sizeof(int)*5);
		}
	}
	//
    return timestamp;
}

void DevicesModule::send_wheel_data(WheelSensorData d)
{
    long timestamp_us = d.timestamp;
    long data = d.data;
    if (d.type == WHEEL_DATA_TYPE_LEFT)
    {
        if (timestamp_us <= last_left_timestamp)
        {
            return;
        }
        if (last_left_data != 0)
        {
            if (data > last_left_data + odometer_input_range ||
                data < last_left_data - odometer_input_range)
            {
                return;
            }
        }
        last_left_data = data;
        last_left_timestamp = timestamp_us;
    }
    else
    {
        if (timestamp_us <= last_right_timestamp)
        {
            return;
        }
        if (last_right_data != 0)
        {
            if (data > last_right_data + odometer_input_range ||
                data < last_right_data - odometer_input_range)
            {
                return;
            }
        }
        last_right_data = data;
        last_right_timestamp = timestamp_us;
    }

    if (p_msg_queue != nullptr)
    {
        std::vector<char> tmp;
        d.to_char_array(&tmp);
        p_msg_queue->send(CHANNEL_WHEEL, tmp.data(), tmp.size());
    }
};

int DevicesModule::handle_movement_message(int speed, int turn_left)
{
    int arr[2];
    pthread_mutex_lock(&mutex);
    printf("Movement %d, %d\n", speed, turn_left);

    if (speed > 2700)
        speed = 2700;
	if (speed < -2700)
       speed = -2700;
	if(turn_left>700) turn_left =700;
	if(turn_left<-700) turn_left =-700;

    arr[0] = (speed - turn_left) / 2;
    arr[1] = (speed + turn_left) / 2;
    char out[100];
    out[0] = 0x02;
    out[1] = 0x09;
    out[2] = 0x01;
    out[3] = (arr[0] >= 0) ? 1 : 0;
    int tmp = (arr[0] >= 0) ? arr[0] : 0 - arr[0];
    out[4] = tmp / 256;
    out[5] = tmp % 256;
    out[6] = (arr[1] >= 0) ? 1 : 0;
    tmp = (arr[1] >= 0) ? arr[1] : 0 - arr[1];
    out[7] = tmp / 256;
    out[8] = tmp % 256;
    out[9] = 0;
    for (int i = 1; i < 9; i++)
    {
        out[9] += out[i];
    }
    out[10] = 0;
    write(move_fd, out, 10);
    pthread_mutex_unlock(&mutex);

    char hex2str[300];

    memset(hex2str, 0, 300);
    for (int j = 0; j < 10; j++)
    {
        sprintf(hex2str + j * 3, "%02x ", out[j]);
    }
    printf("###send to xiaotong %d bytes:%s\n", 10, hex2str);
}
//-180~180
int DevicesModule::handle_nav_message(double startx, double starty, double starttheta, double endx, double endy, double endtheta,int dir)
{
    printf("###clm.DevicesModule::handle_nav_message,@start.x=%lf,y=%lf,theta=%lf,@end.x=%lf,y=%lf,theta=%lf,dir=%s\n",startx,starty,starttheta,endx,endy,endtheta,dir==0?"正行":"倒行");
    int arg1, arg2, arg3, arg4, arg5, arg6;
    arg1 = (int)(startx * 1000);
    arg2 = (int)(starty * 1000);
    arg3 = (int)(starttheta * 10);
    arg4 = (int)(endx * 1000);
    arg5 = (int)(endy * 1000);
    arg6 = (int)(endtheta * 10);

    pthread_mutex_lock(&mutex);
    char out[100];
    out[0] = 0xa0;
    out[1] = 0xb0;
    out[2] = 0x00;
    out[3] = 0x00; //总字节数,高字节
    out[4] = 32;   //总字节数,低字节
    int len = 0;
    out[5] = 0xc1; //0xc100,行走任务
    out[6] = 0x00;

    out[7] = (char)((arg1>>24)&0xff);
    out[8] = (char)((arg1>>16)&0xff);
    out[9] = (char)((arg1>>8)&0xff);
    out[10] = (char)((arg1>>0)&0xff);

    out[11] = (char)((arg2>>24)&0xff);
    out[12] = (char)((arg2>>16)&0xff);
    out[13] = (char)((arg2>>8)&0xff);
    out[14] = (char)((arg2>>0)&0xff);

    out[15] = (char)((arg3>>8)&0xff);
    out[16] = (char)((arg3>>0)&0xff);

    out[17] = (char)((arg4>>24)&0xff);
    out[18] = (char)((arg4>>16)&0xff);
    out[19] = (char)((arg4>>8)&0xff);
    out[20] = (char)((arg4>>0)&0xff);

    out[21] = (char)((arg5>>24)&0xff);
    out[22] = (char)((arg5>>16)&0xff);
    out[23] = (char)((arg5>>8)&0xff);
    out[24] = (char)((arg5>>0)&0xff);

    out[25] = (char)((arg6>>8)&0xff);
    out[26] = (char)((arg6>>0)&0xff);

    out[27] = (dir==0)?0x01:0x04;
	//方向 1正4倒
	
    out[28] = 0x01; //速度h00C8
    out[29] = 0xF4; //速度l
    out[30] = 0x00; //避障范围
    out[31] = 0x00; //校验位

    out[32] = 0x00;
    //A0 B0 00 00 20 C1 00 00 00 10 00 00 00 10 00 00 00 00 00 20 00 00 00 20 00 00 00 01 00 30 00 00
    char hex2str[300];
    memset(hex2str, 0, 300);
    //	sprintf(hex2str,"0a 0b ");
    for (int j = 0; j < 32; j++)
    {
        sprintf(hex2str + j * 3, "%02x ", out[j]);
    }
    printf("send to xiaotong :%s\n", hex2str);

    write(nav_fd, out, 32);

    pthread_mutex_unlock(&mutex);
}

int DevicesModule::handle_turn_message(int dir, double angle)
{
    int arg1;
    arg1 = (int)(angle);
    printf("###clm.DevicesModule::handle_turn_message,dir=%d,angle=%lf\n",dir,angle);

    pthread_mutex_lock(&mutex);
    char out[100];
    out[0] = 0xa0;
    out[1] = 0xb0;
    out[2] = 0x00;
    out[3] = 0x00; //总字节数,高字节
    out[4] = 13;   //总字节数,低字节
    out[5] = 0xc1; //0xc10a,转向任务
    out[6] = 0x0a;

    out[7] = arg1 / 0x100;
    out[8] = arg1 % 0x100;

    out[9] = (char)dir; //方向7顺,8逆
    out[10] = 0x00;     //速度h
    out[11] = 0x32;     //速度l
    out[12] = 0x00;     //校验位

    out[13] = 0x00;

    char hex2str[300];
    memset(hex2str, 0, 300);
    //	sprintf(hex2str,"0a 0b ");
    for (int j = 0; j < 13; j++)
    {
        sprintf(hex2str + j * 3, "%02x ", out[j]);
    }
    printf("send to xiaotong :%s\n", hex2str);
    write(nav_fd, out, 13);

    pthread_mutex_unlock(&mutex);
}

int DevicesModule::handle_collision_message(int level)
{
    printf("###clm.DevicesModule::handle_collision_message,level=%d\n",level);

    pthread_mutex_lock(&mutex);
    char out[100];
	
    out[0] = 0x02;
    out[1] = 0x03;
    out[2] = 0x04;
    out[3] = (char)level;
	out[4] = 0x00;
	
	
    char hex2str[300];
    memset(hex2str, 0, 300);
    for (int j = 0; j < 5; j++)
    {
        sprintf(hex2str + j * 3, "%02x ", out[j]);
    }
    printf("send to xiaotong :%s\n", hex2str);
    write(move_fd, out, 5);

    pthread_mutex_unlock(&mutex);
}
int DevicesModule::handle_lift_message(int dir)
{
    printf("###clm.DevicesModule::handle_lift_message,dir=%d\n",dir);

    pthread_mutex_lock(&mutex);
    char out[100];
	
    out[0] = 0xa0;
    out[1] = 0xb0;
    out[2] = 0x00;
    out[3] = 0x00;
	out[4] = 0x09;
	
    out[5] = 0xc1;
    out[6] = (dir==0)?0x06:0x05;//05:上升,06:下降
    out[7] = 0x00;
	out[8] = 0x01;

    char hex2str[300];
    memset(hex2str, 0, 300);
    for (int j = 0; j < 9; j++)
    {
        sprintf(hex2str + j * 3, "%02x ", out[j]);
    }
    printf("send to xiaotong :%s\n", hex2str);
    write(nav_fd, out, 9);

    pthread_mutex_unlock(&mutex);
}
int DevicesModule::handle_lift_height_message(int height)
{
    printf("###clm.DevicesModule::handle_lift_height_message, height=%d\n",height);

    pthread_mutex_lock(&mutex);
    char out[100];
	
    out[0] = 0xa0;
    out[1] = 0xb0;
    out[2] = 0x00;
    out[3] = 0x00;
	out[4] = 0x0e;
	
    out[5] = 0xc1;
    out[6] = 0x0d;
	int v=height*10000;
    out[7] = 	(char)((v>>24)&0xff);
	out[8] = 	(char)((v>>16)&0xff);
    out[9] = 	(char)((v>>8)&0xff);
	out[10] = 	(char)((v>>0)&0xff);
	
	
    out[11] = 0x0d;
    out[12] = 0x00;
    out[13] = 0x00;

    char hex2str[300];
    memset(hex2str, 0, 300);
    for (int j = 0; j < 14; j++)
    {
        sprintf(hex2str + j * 3, "%02x ", out[j]);
    }
    printf("send to xiaotong :%s\n", hex2str);
    write(nav_fd, out, 14);

    pthread_mutex_unlock(&mutex);
}
int DevicesModule::handle_dio2_message(int angle)	//顶升旋转
{
    printf("###clm.DevicesModule::handle_dio2_message,angle=%d\n",angle);

    pthread_mutex_lock(&mutex);
    char out[100];
	
    out[0] = 0xa0;
    out[1] = 0xb0;
    out[2] = 0x00;
    out[3] = 0x00;
	out[4] = 0x10;
	//
    out[5] = 0xc1;
    out[6] = 0x0b;
    out[7] = 0x00;
	out[8] = 0x00;
    out[9] = 0x00;
    out[10] = angle;
    out[11] = 0x01;
	out[12] = 0x00;
    out[13] = 0x15;
    out[14] = 0x00;
    out[15] = 0xa7;

    char hex2str[300];
    memset(hex2str, 0, 300);
    for (int j = 0; j < 16; j++)
    {
        sprintf(hex2str + j * 3, "%02x ", out[j]);
    }
    printf("send to xiaotong :%s\n", hex2str);
    write(nav_fd, out, 16);

    pthread_mutex_unlock(&mutex);
}

int DevicesModule::open_tty(char const *path, int baudrate, int flags)
{
    int fd;
    speed_t speed;

    if ((speed = getBaudrate(baudrate)) == -1)
    {
        printf("FAILED  to open %s\n", path);
        return -1;
    }
    if ((fd = open(path, O_RDWR | flags)) == -1)
    {
        printf("FAILED  to open %s\n", path);
        return -1;
    }

    struct termios cfg;
    if (tcgetattr(fd, &cfg))
    {
        close(fd);
        printf("FAILED  to open %s\n", path);
        return -1;
    }
    cfmakeraw(&cfg);
    cfsetispeed(&cfg, speed);
    cfsetospeed(&cfg, speed);
    cfg.c_cc[VMIN] = 1;
    cfg.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &cfg))
    {
        close(fd);
        printf("FAILED  to open %s\n", path);
        return -1;
    }

    printf("open %s ok,fd=%d\n", path, fd);
    return fd;
}

speed_t DevicesModule::getBaudrate(int baudrate)
{
    switch (baudrate)
    {
    case 0:
        return B0;
    case 50:
        return B50;
    case 75:
        return B75;
    case 110:
        return B110;
    case 134:
        return B134;
    case 150:
        return B150;
    case 200:
        return B200;
    case 300:
        return B300;
    case 600:
        return B600;
    case 1200:
        return B1200;
    case 1800:
        return B1800;
    case 2400:
        return B2400;
    case 4800:
        return B4800;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default:
        return -1;
    }
}
