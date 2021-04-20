#include <stdio.h>
#include <cmath>
#include "agv.h"
#include "odometer.h"
#include "common/configuration.h"

FILE *fp_angle_imu;
FILE *fp_odom_w;

Odometer::Odometer(MsgQueue *queue, int imu_chan, int wheel_chan, int odom_chan) {
    imu_c = imu_chan;
    wheel_c = wheel_chan;
    odom_c = odom_chan;
    this->queue = queue;
	
	encoder_per_round=get_configs()->get_float("odom", "encoder_per_round", nullptr);
	diameter=get_configs()->get_float("odom", "wheel_diameter", nullptr);
	distance=get_configs()->get_float("odom", "wheel_distance", nullptr);
	
	printf("dia=%f,dis=%f\n",diameter,distance);
	
    pthread_mutex_init(&list_mutex, nullptr);
	left_ready = false;
	right_ready = false;
	first_read = true;
    queue->add_listener(imu_c, this);
    queue->add_listener(wheel_c, this);
	fp_angle_imu = fopen("angle_odo_imu.txt","w");
	fp_odom_w = fopen("odom11.txt","w");


}

Odometer::~Odometer() {
    queue->remove_listener(imu_c, this);
    queue->remove_listener(odom_c, this);
    pthread_mutex_destroy(&list_mutex);
}

int Odometer::handle_wheel_data(WheelSensorData & data) {
    //fprintf(fp_odom_w,"Odom: %ld, %d, %ld\n", data.timestamp, data.type, data.data);
    int type = data.type;
    bool do_update = true;
    double d_theta, k;
	long time_pose_cur;
    pthread_mutex_lock(&list_mutex);
    if(type == WHEEL_DATA_TYPE_LEFT) {
	
        left_datas.push_back(data);
        if(left_datas.size() > 2) {
            left_datas.pop_front();
        }
    }
    if(type == WHEEL_DATA_TYPE_RIGHT) {
        right_datas.push_back(data);
        if(right_datas.size() > 2) {
            right_datas.pop_front();
        }
    }
    //
    // if(++count < skip) {
    //     do_update = false;
    //     // printf("Odom: Skip\n");
    // }
	//printf("11111111odom data received\n");

	if(first_read)
	{
	printf("odom data received\n");
	last_t  =data.timestamp;
                //last_imu = imu_datas.back().angle_z;
	if(type == WHEEL_DATA_TYPE_LEFT)
	{last_left = data.data;left_ready = true;printf("left odom data initilized \n");}
	if(type == WHEEL_DATA_TYPE_RIGHT)
	{last_right =data.data;right_ready = true;printf("right odom data initilized \n");}

	if((left_ready)&&(right_ready)&&imu_ready)
	{first_read = false;left_ready = false;right_ready=false;
		printf("odom data initilized done;");
	}
	else
	do_update = false;
	}

    if(left_datas.size() < 2 ||
        right_datas.size() < 2 ||
        imu_datas.size() < 2 ) {
            do_update = false;
		
            printf("List size : left %d, right %d, imu %d\n", left_datas.size(), right_datas.size(), imu_datas.size());
            count = 0;
    }
    else
 {
        count = 0;
        if(type == WHEEL_DATA_TYPE_LEFT) {
            cur_t = left_datas.back().timestamp;
            cur_left = left_datas.back().data;
	//printf("0000 left data time=%ld, data=%ld,cur_left=%ld\n",data.timestamp,data.data,cur_left);
            do_update = true;
		left_ready = true;
        }
        if(type == WHEEL_DATA_TYPE_RIGHT)
 	{
            cur_t = right_datas.back().timestamp;
            cur_right = right_datas.back().data;
            do_update = true;
		right_ready = true;

        }
        k = imu_datas.back().angular_velocity_z - imu_datas.front().angular_velocity_z;
        k = k / (imu_datas.back().timestamp_us - imu_datas.front().timestamp_us);
        double w1 = imu_datas.back().angular_velocity_z;
        double w2 = w1;
        w1 += (last_t - imu_datas.back().timestamp_us) * k;
        w2 += (cur_t - imu_datas.back().timestamp_us) * k;
        d_theta = ((w1 + w2) * (cur_t - last_t)) / 2.;
        d_theta = d_theta / 1000000.;
	//if(imu_datas.size() < 2) d_theta =0.0;
	/*k = imu_datas.back().angle_z - imu_datas.front().angle_z;
        k = k / (imu_datas.back().timestamp_us - imu_datas.front().timestamp_us);
        double w1 = imu_datas.back().angle_z;
        double w2 = w1;
        w1 += (last_t - imu_datas.back().timestamp_us) * k;
        w2 += (cur_t - imu_datas.back().timestamp_us) * k;
        d_theta = ((w2 - w1) ) ;*/
	cur_imua = imu_datas.back().angle_z;
	//d_theta = -(cur_imua - last_imua);
	//d_theta =imu_datas.back().angle_z - imu_datas.front().angle_z;
	time_pose_cur = imu_datas.back().timestamp_us;
	
    }
    pthread_mutex_unlock(&list_mutex);
	if((left_ready == false)||(right_ready == false)) do_update = false;
    if(!do_update) {
        //printf("Skip odometer data %ld: Not ready\n", data.timestamp);
        return 0;
    }
	//Position pose_raw = get_global_agv_instance()->get_raw_odom_position();
	

    Position pose(cur_t, 0, 0, 0);
    // printf("Odometer : DR (%ld, %ld, %ld, %ld, %ld, %ld, %.3f)\n", data.timestamp , last_t, left , last_left, right , last_right, d_theta);
	 //fprintf(fp_angle_imu,"%ld	%f	%ld	%f	%f\n",imu_datas.back().timestamp_us,d_theta,cur_t - last_t,cur_imua ,last_imua);
	//printf("fp_angle_imu,	%ld	%f	%ld	%f	%f\n",cur_t,d_theta,cur_t - last_t,cur_imua ,last_imua);

    int ret = do_DR(&pose, cur_t - last_t, cur_left- last_left, cur_right- last_right, d_theta);
        //fprintf(fp_angle_imu,"%ld	%ld	%ld	%ld             %d\n",pose.timestamp,1000000*pose.x,1000000*pose.y,1000000*pose.theta,ret);
	//pose.timestamp = time_pose_cur;
	//pose.theta = (cur_imua - last_imua);
	//pose.theta = (cur_imua - start_mapping_angle) - pose_raw.theta;
    if(ret == 0) {
        // printf("Odometer : Change current pose (%.3f, %.3f, %.3f)\n", pose.x, pose.y, pose.theta);
        std::vector<char> tmp;
        pose.to_char_array(&tmp);
        queue->send(odom_c, tmp.data(), tmp.size());
    }
	/*else
	{
	left_ready = false;
	right_ready = false;
	return ret;
	}*/

   // if(fabs(cur_left- last_left)>encoder_per_round)
//	printf("Odometer : DR (%ld, %ld, %ld,cur_%ld, %ld,%ld,%ld)\n", data.timestamp , last_t, data.data, cur_left,last_left,cur_left- last_left,cur_t - last_t);
    // printf("Odometer return value %d\n", ret);
	last_left = cur_left;
	last_right = cur_right;
	last_imua = cur_imua;
    left_ready = false;
	right_ready = false;
	last_t = cur_t;
    return ret;
}
double Odometer::obtain_imu_angle()
{
	double ret_ang = imu_datas.back().angle_z-start_mapping_angle;
	return ret_ang;
}

void Odometer::recieve_message(const int channel, char *buf, const int size) {
    if(channel == imu_c) {
        ImuData2D data;
        data.from_char_array(buf, size);
        handle_imu_data(data);
        // printf("odom insert imu data %ld\n", data.timestamp_us);
    }
    else if(channel == wheel_c) {
        WheelSensorData data;
        data.from_char_array(buf, size);
        handle_wheel_data(data);
        //printf("odom insert wheel data %ld %d\n", data.timestamp, data.type);
    }
}


long Odometer::linear_interpolation(std::list<WheelSensorData> *datas, long time) {

    double k = datas->back().data - datas->front().data;
    k = k / (datas->back().timestamp - datas->front().timestamp);
    long d = (long)(k * (time - datas->back().timestamp));
    return d + datas->back().data;
}

//180
//600

int Odometer::do_DR(Position *buf, long delta_t, long delta_left, long delta_right, double delta_theta) {

    if(delta_t <= 0) delta_t = 20000;

    double PI = 3.14159265358979;
	if(fabs(delta_left)>encoder_per_round||fabs(delta_right)>encoder_per_round)
	{
	printf("cur delta exceed the total round\n");
	delta_left = 0.0;
    	delta_right = 0.0;
	}
	else
	{
    delta_left = delta_left;
    delta_right = delta_right;
	}
    delta_theta = - delta_theta;

//    double W_left=delta_left*2*PI/(65536*delta_t);
//	double W_right=delta_right*2*PI/(65536*delta_t);
    	double W_left=delta_left*2*PI/(encoder_per_round*delta_t)*1000000;///// chang to line velocity of the left wheel m/s
	double W_right=delta_right*2*PI/(encoder_per_round*delta_t)*1000000;
	double V_left=W_left*diameter/2.;
	double V_right=W_right*diameter/2.;

	double VOdom=(V_left+V_right)/2.;
	
	double WOdom=(V_right-V_left)/distance;
	//if(delta_left>1)
	//if(fabs(VOdom*0.001)>0.2)
	//printf("lvlel=%f,rvel=%f, line_vel=%f,WOdom=%f,diameter=%f,delta_left=%ld,delta_right=%ld,%ld\n",V_left,V_right,VOdom,WOdom,diameter,delta_left,delta_right,delta_t);
	//if(fabs(WOdom*delta_t)>0.0001)
	//车体转过的位姿
	double delta_x=0;
	double delta_y=0;
	if(fabs(V_left*0.001)>0.007||fabs(V_right*0.001)>0.007)
	buf->delt_t = 20000;/////如果机器人左右轮速度都大于0.007，则认为机器人在动，否则认为机器人静止
	else
	buf->delt_t = 0;
	/*if(delta_left == delta_right || fabs(delta_theta)<0.000000001)////delta_theta ==0 >>>>fabs(delta_theta)<0.000000001
	{
		delta_theta=0;
		delta_x=VOdom*delta_t*cos(buf->theta)/1000*0.000001;
		delta_y=VOdom*delta_t*sin(buf->theta)/1000*0.000001;

	}
	else*/
	{
		// delta_theta = delta_theta * 0.8 + (WOdom*delta_t) * 0.2
		//if(fabs(delta_theta / delta_t*500000)<M_PI)/////避免imu反馈角度突变
		if(buf->delt_t == 0)////如果里程计运动缓慢，则取里程计的角速度数据，否则取陀螺仪的角速度数据
		delta_theta =  (WOdom*delta_t)*0.000001;
		else
	        WOdom = delta_theta / delta_t;
		
		
		//else
		//delta_theta =  (WOdom*delta_t)*0.000001;
		/*if(fabs(delta_theta)>0.000001)
		{
		delta_y=VOdom*(cos(buf->theta)-cos(buf->theta+delta_theta))/(WOdom*1000)*0.000001;
		delta_x=VOdom*(sin(buf->theta+delta_theta)-sin(buf->theta))/(WOdom*1000)*0.000001;
		}
		else*/
		{
		//delta_theta =  (WOdom*delta_t)*0.000001;///LWG
		delta_x=VOdom*delta_t*cos(delta_theta)/1000*0.000001;
		delta_y=VOdom*delta_t*sin(delta_theta)/1000*0.000001;
		}
	}
	if(isnanl(delta_x)||isnanl(delta_y)) 
	{delta_x =0.0;delta_y=0.0;printf("the number is not a number\n");}
	//if(fabs(V_left)>0.007||fabs(V_right)>0.007)
	//printf("V_right=%f,V_right=%f,delt_t=%d,delta_theta=%f\n",V_left,V_right,delta_t,delta_theta*1000000);
	buf->x+=delta_x;
	buf->y+=delta_y;
	buf->theta += delta_theta;
 /*   while(buf->theta > PI) {
        buf->theta -= (PI * 2);
    }
    while(buf->theta <= -PI) {
        buf->theta += (PI * 2);
    }*/
	//fprintf(fp_angle_imu,"%ld	%f	%ld	%f	%f\n",buf->timestamp,delta_theta,delta_t,delta_x,delta_y);
	//if(fabs(delta_t)>22000||fabs(delta_t)<19000)
	//delta_t = 20000;

	//buf->timestamp += delta_t;

	

	
    return 0;
}

int Odometer::handle_imu_data(ImuData2D & data) {
    
    // printf("IMU: %ld, %.3f\n", data.timestamp_us, data.angular_velocity_z);
    pthread_mutex_lock(&list_mutex);
    imu_datas.push_back(data);
	if(first_read)
	{
	printf("imu data received\n");
	{last_imua = data.angle_z;imu_ready = true;printf("imu data initilized \n");}
	}
	if(start_mapping)
	{
	start_mapping = false;
	start_mapping_angle = data.angle_z;
	printf("imu data start_mapping_angle = %f\n");
	}
    if(imu_datas.size() > 2) {
        imu_datas.pop_front();
    }
    pthread_mutex_unlock(&list_mutex);

    return 0;
}
