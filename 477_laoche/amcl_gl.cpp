#include <cmath>
#include <memory>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include "amcl_gl.h"
#include "agv.h"
#include "common/configuration.h"
//query_info my_query_data;/////实时查询数据
//FILE *map_odom_data_file;////未处理地图数据文件
//FILE *map_radar_data_file;////未处理地图数据文件
//FILE *map_odoraw_file;////未处理地图数据文件

typedef struct
{
	// Total weight (weights sum to 1)
	double weight;
	double score;
	// Mean of pose esimate
	pf_vector_t pf_pose_mean;

	// Covariance of pose estimate
	pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

double
normalize(double z)
{
	return atan2(sin(z), cos(z));
}
double
angle_diff(double a, double b)
{
	double d1, d2;
	a = normalize(a);
	b = normalize(b);
	d1 = a - b;
	d2 = 2 * M_PI - fabs(d1);
	if (d1 > 0)
		d2 *= -1.0;
	if (fabs(d1) < fabs(d2))
		return(d1);
	else
		return(d2);
}

Amcl_gl::Amcl_gl(MsgQueue *queue, int radar_chan,int odom_chan)
{
    this->queue_a = queue;
    radar_c = radar_chan;
	odom_c = odom_chan;
    queue_a->add_listener(radar_c, this);
	queue_a->add_listener(odom_c, this);
	pthread_mutex_init(&listen_pose_mutex, nullptr);

    pthread_mutex_init(&mutex_pose, nullptr);
    pthread_mutex_init(&mutex_odom_list, nullptr);

	pthread_mutex_init(&mutex_pose_raw, nullptr);
    pthread_mutex_init(&mutex_odom_list_raw, nullptr);
	map_loaded_success = false;
    para_initial();
}
Amcl_gl::~Amcl_gl()
{
	pthread_mutex_destroy(&listen_pose_mutex);
	pthread_mutex_destroy(&mutex_pose);
	pthread_mutex_destroy(&mutex_odom_list);
	pthread_mutex_destroy(&mutex_pose_raw);
	pthread_mutex_destroy(&mutex_odom_list_raw);
	queue_a->remove_listener(radar_c, this);
	queue_a->remove_listener(odom_c, this);
		if (map_ != NULL) {
			map_free(map_);
			map_ = NULL;
			//my_globalplanner.~GlobalPlanner();
		}
		if (pf_ != NULL) {
			pf_free(pf_);
			pf_ = NULL;
		}
		if (odom_ != NULL) {
			delete odom_;
			odom_ = NULL;
		}
		if (laser_ != NULL) {
			delete laser_;
			laser_ = NULL;
		}
}

void Amcl_gl::add_odom_data_raw(Position &pose_change)
{
    long timestamp = pose_change.timestamp;
    pthread_mutex_lock(&mutex_pose_raw);
    if (timestamp < pose2d_raw.timestamp)
    {
	printf("000000timestamp is too small  exit\n");
        //pthread_mutex_unlock(&mutex_pose_raw);
        //return;
    }
    pthread_mutex_lock(&mutex_odom_list_raw);
    if (odom_list_raw.empty())
    {
        odom_list_raw.push_back(pose_change);
    }
    else
    {
        for (auto iter = odom_list_raw.begin();; iter++)
        {
            if (iter == odom_list_raw.end() || iter->timestamp >= timestamp)
            {
                odom_list_raw.insert(iter, pose_change);
                break;
            }
        }
    }
    pthread_mutex_unlock(&mutex_odom_list_raw);
    pthread_mutex_unlock(&mutex_pose_raw);
}
void Amcl_gl::_begin_map_create()
{	
	record_order = 1;
	first_located = false;
	first_record = true;
	begin_mapping = true;
	flush_odom_data_raw();
	pose2d_raw.x = 0;
	pose2d_raw.y = 0;
	pose2d_raw.theta = 0;
	last_record_pos = pose2d_raw;
	map_raw_data_file = fopen("map.rawmap","w+");
	usleep(1500000);
	fprintf(map_raw_data_file,"begin mapping\n");
	//map_odom_data_file = fopen("map_odom_data_file.txt","w");////未处理地图数据文件
	//map_radar_data_file = fopen("map_radar_data_file.txt","w");////未处理地图数据文件
	//map_odoraw_file = fopen("map_odomraw_file.txt","w");////未处理地图数据文件
	
}
void Amcl_gl::_end_map_create()
{
	begin_mapping = false;
	fclose(map_raw_data_file);
	//fclose(map_odom_data_file);
	//fclose(map_radar_data_file);

}

void Amcl_gl::add_odom_data(Position &pose_change)
{
    long timestamp = pose_change.timestamp;
    pthread_mutex_lock(&mutex_pose);
    if (timestamp < pose2d.timestamp)
    {
	printf("111111timestamp is too small  exit\n");
        //pthread_mutex_unlock(&mutex_pose);
        //return;
    }
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
void Amcl_gl::update_odom_list_timestamp_raw(long timestamp_us)
{
    pthread_mutex_lock(&mutex_odom_list_raw);
    while (!odom_list_raw.empty())
    {
        if (odom_list_raw.front().timestamp > timestamp_us)
        {
            break;
        }
        odom_list_raw.pop_front();
    }
    pthread_mutex_unlock(&mutex_odom_list_raw);
}
void Amcl_gl::update_odom_list_timestamp(long timestamp_us)
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
void Amcl_gl::flush_odom_data()
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
	//printf("fffffffffffffffffffffffffffffffcur pose is theta=%f\nddddddddddddddddddddddddddd",pose2d_raw.theta);
    pthread_mutex_unlock(&mutex_odom_list);
    pthread_mutex_unlock(&mutex_pose);
}
void Amcl_gl::flush_odom_data_raw()
{
    pthread_mutex_lock(&mutex_pose_raw);
    Position pose = pose2d_raw;
    pthread_mutex_lock(&mutex_odom_list_raw);
    if (pose_level <= 1)
    {
        while (!odom_list_raw.empty())
        {
            pose = pose * odom_list_raw.front();
            odom_list_raw.pop_front();
        }
        pose2d_raw = pose;
    }
    else
    {
        odom_list_raw.clear();
    }
	//printf("qqqqqqqqqqqqqqqqqqqqqqqqqqqqcur pose is theta=%f\nddddddddddddddddddddddddddd",pose2d_raw.theta);
    pthread_mutex_unlock(&mutex_odom_list_raw);
    pthread_mutex_unlock(&mutex_pose_raw);
}
Position Amcl_gl::expolate_current_position_tim(long timestamp)
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
	pose2d = ret;
	while (!odom_list.empty())
    {
        if (odom_list.front().timestamp > timestamp)
        {
            break;
        }
        odom_list.pop_front();
    }
	pthread_mutex_unlock(&mutex_odom_list);
	pthread_mutex_unlock(&mutex_pose);
	return ret;
}
//最新的amcl结果,当前位姿要将后面的位姿变化乘上去
void Amcl_gl::update_last_position(Position &pose)
{
    pthread_mutex_lock(&mutex_pose);
        if (pose.timestamp > pose2d.timestamp)
        {
            pose2d = pose;
            update_odom_list_timestamp(pose.timestamp);
        }
    pthread_mutex_unlock(&mutex_pose);
}

Position Amcl_gl::expolate_current_position_tim_raw(long timestamp)
{
	pthread_mutex_lock(&mutex_pose_raw);
	Position ret = pose2d_raw;

	pthread_mutex_lock(&mutex_odom_list_raw);
	for (Position &p : odom_list_raw)
	{
		if (p.timestamp > ret.timestamp && (p.timestamp <= timestamp))
		{
			ret = ret * p;
		}

	}
	pose2d_raw = ret;
	while (!odom_list_raw.empty())
    {
        if (odom_list_raw.front().timestamp > timestamp)
        {
            break;
        }
        odom_list_raw.pop_front();
    }
	pthread_mutex_unlock(&mutex_odom_list_raw);
	pthread_mutex_unlock(&mutex_pose_raw);
	return ret;
}
Position Amcl_gl::expolate_current_position()
{
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
Position Amcl_gl::expolate_current_position_raw()
{
    pthread_mutex_lock(&mutex_pose_raw);
    Position ret = pose2d_raw;
    pthread_mutex_lock(&mutex_odom_list_raw);
    for (Position &p : odom_list_raw)
    {
        if (p.timestamp > ret.timestamp)
        {
            ret = ret * p;
        }
    }
    pthread_mutex_unlock(&mutex_odom_list_raw);
    pthread_mutex_unlock(&mutex_pose_raw);
    return ret;
}
void Amcl_gl::_recor_data2map()
{ 
	if(!begin_mapping) return;
	double xp,yp,laser_len;
	if (fabs(last_record_pos.x - radar_pos.x)>0.05||fabs(last_record_pos.y - radar_pos.y)>0.05||fabs(last_record_pos.theta - radar_pos.theta)>0.1||first_record)
	{
	fprintf(map_raw_data_file,"%d\t",record_order);
	if(first_record) first_record = false;
	//{fprintf(map_raw_data_file,"%f\t%f\t%f\t%d\t", 0,0,0,381);first_record = false;}
	//else
	fprintf(map_raw_data_file,"%f\t%f\t%f\t%d\t", radar_pos.x,radar_pos.y,radar_pos.theta,381);
	//fprintf(map_raw_data_file,"%f\t%f\t%f\t%d\t", radar_pos.x,radar_pos.y,radar_pos.theta,last_radar_data.points.size());

	for(int i=21;i<402;i++)
	//for(int i=0;i<last_radar_data.points.size();i++)
	{
	xp = last_radar_data.points[i](0);
	yp = last_radar_data.points[i](1);
	laser_len = sqrt(xp*xp+yp*yp);
	fprintf(map_raw_data_file,"%f,",laser_len);
	}
	fprintf(map_raw_data_file,"\n");
	record_order++;
	last_record_pos = radar_pos;
	}
}
void Amcl_gl::_recor_data2xm()
{
	/*if(!begin_mapping) return;
	double xp,yp,laser_len;

	//fclose(map_odom_data_file);
	//fclose(map_radar_data_file);

	fprintf(map_odom_data_file,"%d ",record_order);
	fprintf(map_radar_data_file,"%d ",record_order);
	xp = radar_pos.x+0.361*cos(radar_pos.theta);
	yp = radar_pos.x+0.361*sin(radar_pos.theta);

	fprintf(map_odom_data_file,"%d %f %f %f\n",radar_pos.timestamp, xp,yp,radar_pos.theta);
	fprintf(map_radar_data_file,"%d %d",last_radar_data.timestamp,last_radar_data.points.size());

	for(int i=0;i<last_radar_data.points.size();i++)
	{
	xp = last_radar_data.points[i](0);
	yp = last_radar_data.points[i](1);
	laser_len = sqrt(xp*xp+yp*yp);
	fprintf(map_radar_data_file,"%f,",laser_len);
	}
	fprintf(map_radar_data_file,"\n");
	record_order++;
	*/
}

void Amcl_gl::recieve_message(const int channel, char *buf, const int size)
{   
	Position odom_temp;
    pthread_mutex_lock(&listen_pose_mutex);
     if (channel == CHANNEL_RADAR)
	 {//得到scanmatch
        PointCloudData data(0);
        data.from_char_array(buf, size);
		//radar_pos = get_global_agv_instance()->expolate_current_position_tim(data.timestamp);
		radar_pos = expolate_current_position_tim_raw(data.timestamp);
		 odom_temp = expolate_current_position_raw();
		
		last_radar_data = data;
		//_recor_data2map();
		//_recor_data2xm();
		//if(begin_mapping) fprintf(map_odoraw_file,"%d %f %f %f\n",odom_temp.timestamp, odom_temp.x,odom_temp.y,odom_temp.theta);
       		if(first_located )////如果初始定位已经完成，启动实时定位线程
		{
			if(robot_stop_cnt<70)
			{
			particle_filter_run(true);////////启动粒子滤波器
			// printf("odom x=%f,y=%f,theta=%f,robot_stop_cnt=%d\n",odom_temp.x,odom_temp.y,odom_temp.theta,robot_stop_cnt);
			//update_last_position(locate_pose);
			;
			}
			//
		}

		laser_recv_cnt++;
		if(laser_recv_cnt >= 10)
		{
		laser_recv_cnt = 0;
		}
	}
	else if (channel == CHANNEL_ODOM)
    {
        Position pose;
        pose.from_char_array(buf, size);
		//pose.y = -pose.y;
		//pose.theta = -pose.theta;
		add_odom_data_raw(pose);
		add_odom_data(pose);
		//

		//printf("---------%ld %f %f %f\n",pose.delt_t, pose.x*1000000,pose.y*1000000,pose.theta*1000000);
		if(pose.delt_t<=0)
		{if(robot_stop_cnt<65535)robot_stop_cnt++;}
		else
		robot_stop_cnt = 0;
		last_time_pose = pose.timestamp;

	//if (get_current_map().is_nullptr())////防止scan-matching没有启动，导致odom_list不断插入数据，占用cpu过高
		if(odom_list.size()>200) flush_odom_data();
		if(odom_list_raw.size()>200) flush_odom_data_raw();
    }
       pthread_mutex_unlock(&listen_pose_mutex);
}
bool Amcl_gl::para_initial()
{
	min_particles_ = 500;////允许的粒子数量的最小值，默认100(500)
	max_particles_ = 5000;////允许的例子数量的最大值，默认5000
	kld_err = 0.009;    //真实分布和估计分布之间的最大误差，默认0.01(0.05)
	kld_z = 0.99;  //上标准分位数（1-p），其中p是估计分布上误差小于kld_err的概率，默认0.99
	d_thresh_ = 0.1;//////粒子滤波触发条件1(0.2)
	a_thresh_ = 0.1;//////粒子滤波触发条件2(0.5)
	resample_interval_ = 3;////重采样间隔(2)
	recovery_alpha_slow = 0.001; //慢速的平均权重滤波的指数衰减频率，用作决定什么时候通过增加随机位姿来recover，默认0（disable），可能0.001是一个不错的值
	recovery_alpha_fast = 0.1;  //快速的平均权重滤波的指数衰减频率，用作决定什么时候通过增加随机位姿来recover，默认0（disable），可能0.1是个不错的值
	save_pose_rate = 2;  //存储上一次估计的位姿和协方差到参数服务器的最大速率。被保存的位姿将会用在连续的运动上来初始化滤波器。-1.0失能。(0.5)


	/////激光模型参数
	laser_min_range = 0.3;  //被考虑的最小扫描范围；参数设置为-1.0时，将会使用激光上报的最小扫描范围
	laser_max_range = 19;  //被考虑的最大扫描范围；参数设置为-1.0时，将会使用激光上报的最大扫描范围
	laser_max_beams = 60;  //更新滤波器时，每次扫描中多少个等间距的光束被使用（减小计算量，测距扫描中相邻波束往往不是独立的可以减小噪声影响，太小也会造成信息量少定位不准）
	laser_z_hit = 0.95; //(0.5)模型的z_hit部分的混合权值，默认0.95(混合权重1.具有局部测量噪声的正确范围--以测量距离近似真实距离为均值，其后laser_sigma_hit为标准偏差的高斯分布的权重)
	laser_z_short = 0.02; //(0.05)模型的z_short部分的混合权值，默认0.1（混合权重2.意外对象权重（类似于一元指数关于y轴对称0～测量距离（非最大距离）的部分：
	laser_z_max = 0.002; //0.05模型的z_max部分的混合权值，默认0.05（混合权重3.测量失败权重（最大距离时为1，其余为0）
	laser_z_rand = 0.05; //0.5模型的z_rand部分的混合权值，默认0.05（混合权重4.随机测量权重--均匀分布（1平均分布到0～最大测量范围）
	laser_sigma_hit = 0.05; //0.2被用在模型的z_hit部分的高斯模型的标准差，默认0.2m
	laser_lambda_short = 0.5; //0.1模型z_short部分的指数衰减参数，默认0.1（根据ηλe^(-λz)，λ越大随距离增大意外对象概率衰减越快）
	laser_likelihood_max_dist = 0.1; //2.0地图上做障碍物膨胀的最大距离，用作likelihood_field模型
	laser_model_type = LASER_MODEL_LIKELIHOOD_FIELD; //LASER_MODEL_LIKELIHOOD_FIELD//likelihood_field模型使用，可以是beam, likehood_field, likehood_field_prob
	do_beamskip_ = 0; //////false
	beam_skip_distance_ = 0.5; ///0.5
	beam_skip_threshold_ = 0.3; ///0.3
	//里程计模型参数
	odom_model_t odom_model_type; //ODOM_MODEL_DIFF//diff模型使用，可以是"diff", "omni", "diff-corrected", "omni-corrected",后面两 
	odom_alpha1 = 0.2; //0.2指定由机器人运动部分的旋转分量估计的里程计旋转的期望噪声，默认0.2
	odom_alpha2 = 0.2; //0.2制定由机器人运动部分的平移分量估计的里程计旋转的期望噪声，默认0.2
	odom_alpha3 = 0.1; //0.8指定由机器人运动部分的平移分量估计的里程计平移的期望噪声，默认0.2
	odom_alpha4 = 0.1; //0.2指定由机器人运动部分的旋转分量估计的里程计平移的期望噪声，默认0.2
	odom_alpha5; //0.1平移相关的噪声参数（仅用于模型是“omni”的情况--wiki官网的注释）

	////////////////////////////////
	resample_count_;///////重采样计数
	double offset_x = get_configs()->get_float("radar", "radar_position_x", nullptr);
	double offset_y = get_configs()->get_float("radar", "radar_position_y", nullptr);
	int from_index = get_configs()->get_int("radar", "first_point_index", nullptr);
	int to_index = get_configs()->get_int("radar", "last_point_index", nullptr);
	ini_ang = from_index;
	end_ang = to_index;
	ang_step = 0.5;
	//resample_interval_ = 2;
	pf_init_ = false;
	para_LASER_2_BASE_X = offset_x;
	para_LASER_2_BASE_Y = offset_y;
	para_LASER_2_BASE_A = 0;
	radar_position.x = para_LASER_2_BASE_X;
	radar_position.y = para_LASER_2_BASE_Y;
	radar_position.theta = para_LASER_2_BASE_A;
	locate_step = 0;

	first_located = false;
}
bool Amcl_gl::set_laser_para(Position radarpos,float radar_ini_ang,float radar_end_ang,float radar_step,float radar_max_range,float radar_min_range)
{
	ini_ang = radar_ini_ang;
	end_ang = radar_end_ang;
	ang_step = radar_step;
	if(radar_min_range>0)
	laser_min_range = radar_min_range;
	if(radar_max_range>0)
	laser_max_range = radar_max_range;

	para_LASER_2_BASE_X = radarpos.x;
	para_LASER_2_BASE_Y = radarpos.y;
	para_LASER_2_BASE_A = radarpos.theta;
}
map_t* Amcl_gl::do_map_open()
{
	map_t* map = map_alloc();

	int index_x;
	int index_y;
	int point_count=0;
	float max_x,max_y;
	float min_x,min_y;
	float threshold = 0.6;
	int width = gridmap->map_info.width;
	int height = gridmap->map_info.height;
	printf("width=%d,height=%d",width ,height);
	map->scale = gridmap->map_info.resolution;
	 printf("map->resolution:%f\t",map->scale);
	map->size_x = width ;
	 printf("map->size_x:%d\t",map->size_x);
	if(map->size_x<=0) return map;
	map->size_y = height;
	  printf("map->size_y:%d\t",map->size_y);
	if(map->size_y<=0) return map;

	float m_scale = map->scale;/////缩放因子

	min_x = gridmap->map_info.origen_x;
	min_y = gridmap->map_info.origen_y;
	printf("min_x=%ftttooootttmin_y=%f\t",min_x,min_y);

	map->origin_x = (map->size_x / 2) * m_scale + min_x;
	map->origin_y = (map->size_y / 2) * m_scale + min_y;
	//map->origin_y = (map->size_y / 2) * m_scale - (min_y+map->size_y*m_scale);
	
	printf("origin_x=%f,origin_y=%f\n",map->origin_x,map->origin_y);

  	map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
	
	char grid_value;
	int xi, yi;
	
	for(int i=0;i<map->size_x * map->size_y;i++)
	{
	map->cells[i].occ_state = -1;/////先全部初始化为n障碍
	//local_map->datas[i] = 0;
	}
	int total_height = height-1;
	//for (index_y = total_height; index_y>=0; index_y--)
	for (index_y = 0; index_y<height; index_y++)
	{
	for (index_x = 0; index_x<width; index_x++)
	{
		//xi = index_x*m_scale; yi = index_y*m_scale;
		xi = index_x; yi = index_y;
		//grid_value = gridmap->get(xi, yi);
		grid_value = gridmap->datas[gridmap->get_info().map_xy_to_array_index(xi, yi)];
		//if (grid_value!=255)
		//printf("%d ",grid_value);
		if (grid_value>20&&grid_value<255)
		{
		//map->cells[-(index_y-total_height)*map->size_x + index_x].occ_state = -1;
		//map->cells[(total_height-index_y)*width + (index_x)].occ_state = 1;
		map->cells[(index_y)*width + (index_x)].occ_state = 1;
		//printf("i=%d,j=%d,x=%f,y=%f>>>   ",xi,yi,MAP_WXGX(map, xi),MAP_WYGY(map, yi));
		//local_map->datas[index_y*map->size_x + index_x] = 99;
		}
		else if (grid_value==255)
		//map->cells[(total_height-index_y)*width + (index_x)].occ_state = 0;
		map->cells[(index_y)*width + (index_x)].occ_state = 0;



	}
	//printf("\n");
	}

	
	map_loaded_success = true;
printf("地图已加载完毕>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
  return map;
}

int Amcl_gl::particle_filter_initialize()///////初始化粒子滤波器
{
  // Create the particle filter
 	pf_ = pf_alloc(min_particles_, max_particles_,
                 recovery_alpha_slow, recovery_alpha_fast,
                 (pf_init_model_fn_t)uniformPoseGenerator,
                 (void *)map_);
  	pf_->pop_err = kld_err;
  	pf_->pop_z = kld_z;
  // Initialize the filter
///////////////////////////////////////////////////////////
	 pf_vector_t pf_init_pose_mean = pf_vector_zero();
	  pf_init_pose_mean.v[0] = init_pos_x;
	int j;
	float ini_y=init_pos_y;
	j = MAP_GYWY(map_,ini_y );
	  //pf_init_pose_mean.v[1] = MAP_WYGY(map_, map_->size_y-j);
	  //pf_init_pose_mean.v[2] = -init_pos_theta;
	  pf_init_pose_mean.v[1] = init_pos_y;
	  pf_init_pose_mean.v[2] = init_pos_theta;
		flush_odom_data_raw();
		pose2d_raw.x = init_pos_x;
		pose2d_raw.y = init_pos_y;
		pose2d_raw.theta = init_pos_theta;
	usleep(200000);////
	  printf("the initial pos x=%f,y=%f,theta=%f\n",init_pos_x,init_pos_y,init_pos_theta);
	  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
	  pf_init_pose_cov.m[0][0] = 0.2;
	  pf_init_pose_cov.m[1][1] = 0.2;
	  pf_init_pose_cov.m[2][2] = 0.1;
	  if((fabs(init_pos_x)>fuzzy_1)||(fabs(init_pos_y)>fuzzy_1))
	 { pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);	
	printf("Initializing with uniform distribution");}
	else
{ pf_init_model(pf_, (pf_init_model_fn_t)uniformPoseGenerator,  (void *)map_);
printf("Global initialisation done!");}
	pf_init_ = false;
	// Instantiate the sensor objects
  // Odometry
  	delete odom_;
  	odom_ = new AMCLOdom();
	assert(odom_);
  	odom_->SetModelDiff(odom_alpha1, odom_alpha2, odom_alpha3, odom_alpha4);
// Laser
 	delete laser_;
 	laser_ = new AMCLLaser(laser_max_beams, map_);
 	assert(laser_);
	printf("Initializing likelihood field model; this can take some time on large maps...\n");
	laser_->SetModelLikelihoodField(laser_z_hit, laser_z_rand, laser_sigma_hit,
                                    laser_likelihood_max_dist);	
	printf("Done initializing likelihood field model.\n");
	printf("粒子滤波器laser_z_hit=%f,z_rand=%f,sigma_hit=%f>>>>>>>>>>>>>>>>>>\n",laser_z_hit, laser_z_rand, laser_sigma_hit);
	printf("粒子滤波器，odo,激光初始化函数运行完毕>>>>>>>>>>>>>>>>>>>\n");
	return 1;
}
void Amcl_gl::set_init_pos(float x,float y,float theta)
{
	//int j;
	//j = MAP_GYWY(map_, y);
	init_pos_x = x;
	init_pos_y = y;
	//init_pos_y = MAP_WYGY(map_, map_->size_y-j;
	init_pos_theta = theta;
}
pf_vector_t
uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;

  double min_x, max_x, min_y, max_y;

  min_x = map->origin_x - (map->size_x * map->scale)/2.0 ;
  max_x = map->origin_x + (map->size_x * map->scale)/2.0 ;
  min_y = map->origin_y - (map->size_y * map->scale)/2.0 ;
  max_y = map->origin_y + (map->size_y * map->scale)/2.0 ;
//	min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  //max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
 // min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  //max_y = (map->size_y * map->scale)/2.0 + map->origin_y;
  //printf("粒子均布函数运行..sizex=%d..sizey=%d..reso=%f..minx=%f,min_y=%f,max_x=%f,max_y=%f\n",map->size_x,map->size_y,map->scale,min_x,min_y,max_x,max_y);
/*	MapInfo map_info;
	map_info.width =map->size_x;
	map_info.height =map->size_y/2;
	map_info.resolution =map->scale;

	SimpleGridMap *local_map = new SimpleGridMap(map_info);	//临时构建的地图
	local_map->datas.resize(map->size_x*map->size_y/2);
	for(int j=0;j<map->size_x * map->size_y/2;j++)
	local_map->datas.push_back(0);

      	for(int i=0;i<map->size_x * map->size_y/2;i++)
	{
	if(map->cells[i].occ_state == -1)
	local_map->datas[i] = 99;
	else if(map->cells[i].occ_state == 1)
	local_map->datas[i] = 0;
	else 
	local_map->datas[i] = 10;
	//printf("%d ",map->cells[i].occ_state);//map->cells[i].occ_state = -1;
	}
	std::vector<char> map_data;
	local_map->to_char_array(&map_data);
    std::ofstream ofile("local_locating_map7");
    ofile.write(map_data.data(), map_data.size());
    ofile.close();
	for(int jj=0;jj< map->size_y;jj++)
	for(int ii=0;ii< map->size_x ;ii++)
	{
	if(map->cells[MAP_INDEX(map,ii,jj)].occ_state == -1)
	printf("  >>> ii=%d,jj=%d  >>> ",ii,jj);//map->cells[i].occ_state = -1;
	}*/

  pf_vector_t p;
     int ii=0;
  for(;;)
  {
	ii++;
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
	//printf("i=%d,j=%d  ",i,j);
	//printf("ii=%d,x=%f,y=%f,occ=%d",ii,p.v[0],p.v[1],map->cells[MAP_INDEX(map,i,j)].occ_state);
    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }
	//printf("粒子均匀分布函数运行完毕>>>>>>>>>>>>>>>>>>>\n");
  return p;
}
bool Amcl_gl::particle_filter_run(char fup)////////启动粒子滤波器
{

//////////////////////////////////////////////////////////定义粒子滤波所需局部变量
	pf_vector_t delta;
	pf_vector_t base_pos_pre;////定位算法执行之前的当前机器人位置
	bool force_publication;
	bool resampled;
	double angle_min;
	double angle_increment;
	double range_min;
	double max_weight = 0.0;
	int max_weight_hyp = -1;
	double weight;
	bool update ;
	double laser_len;
	char resample_upd = 0;	
	pf_sample_set_t* set;	
	delta = pf_vector_zero();
	pf_vector_t GetOdoPose;

	GetOdoPose = pf_vector_zero();
	GetOdoPose.v[0] = radar_pos.x;
	GetOdoPose.v[1] = radar_pos.y;
	GetOdoPose.v[2] = radar_pos.theta;
	long pre_time=get_current_time_us();
	long cc_time;
	if(pf_init_)
	{

		delta.v[0] = GetOdoPose.v[0] - pf_odom_pose_.v[0];
		delta.v[1] = GetOdoPose.v[1] - pf_odom_pose_.v[1];
		delta.v[2] = angle_diff(GetOdoPose.v[2], pf_odom_pose_.v[2]);
		
		// See if we should update the filter
		update = fabs(delta.v[0]) > d_thresh_ ||
			  fabs(delta.v[1]) > d_thresh_ ||
			  fabs(delta.v[2]) > a_thresh_;
		
		if(fup == 1 )update = true;

		// Set the laser update flags
		if(update)
		lasers_update_ = true;
		//printf("计算odo距离部分激活》》》》》。》》》》\n");
	}
	force_publication = false;
	if(!pf_init_)
	{
		// Pose at last filter update
		pf_odom_pose_ = GetOdoPose;
		// Filter is now initialized
		pf_init_ = true;

		// Should update sensor data
		lasers_update_ = true;

		force_publication = true;
		resample_count_ = 0;
		//printf("初始pf——init部分激活》》》》》。》》》》\n");
	}
// If the robot has moved, update the filter
	else if(pf_init_ && lasers_update_)
	{
		//printf("pose\n");
		AMCLOdomData odata;
		odata.pose = GetOdoPose;
		odata.delta = delta;
		// Use the action data to update the filter
		odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);
	}
	resampled = false;
///////////////////////////////////////////////////////////////////////
	// If the robot has moved, update the filter
	if(lasers_update_)
	{
		AMCLLaserData ldata;
		ldata.sensor = laser_;
		ldata.range_count = fabs(end_ang - ini_ang);
		
		angle_increment = ang_step / 180 * M_PI;
		angle_min = -ldata.range_count*ang_step / 2 / 180 * M_PI;
		
		angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;
		ldata.range_max = laser_max_range;
		//printf("angle_min=%f,angle_increment=%f,ldata.range_max=%f",angle_min,angle_increment,ldata.range_max);
		

		range_min = laser_min_range;
		// The AMCLLaserData destructor will free this memory
		
		ldata.ranges = new double[ldata.range_count][2];
		assert(ldata.ranges);
		//printf("激光最大距离=%f激光最小距离=%f>>>>>",ldata.range_max,range_min);
		/////////////////////整理激光数据
		int size = last_radar_data.points.size();
		//double xp, yp;
		if (size > ldata.range_count) size = ldata.range_count;
		for (int i = 0; i<size; i++)
		{
		// amcl doesn't (yet) have a concept of min range.  So we'll map short
		// readings to max range.
			double xp = last_radar_data.points[size-i-1](0);
			double yp = last_radar_data.points[size-i-1](1);
			laser_len = sqrt(xp*xp+yp*yp);
			if (laser_len <= range_min)
		ldata.ranges[i][0] = ldata.range_max;
		else
			ldata.ranges[i][0] = laser_len;
		// Compute bearing
		ldata.ranges[i][1] = angle_min +
		      (i * angle_increment) ;//
		//if(i%5 == 0)
		// printf("#%.2f %.2f##",ldata.ranges[i][0],ldata.ranges[i][1]);
		}

		
		//  printf("=================================================================\n");
		//getchar();
		laser_->UpdateSensor(pf_, (AMCLSensorData*)&ldata);
    		lasers_update_ = false;
		
///////////////////////////////////////////////////保存上一步的位置为激光相对于odo的位置
		// Resample the particles
		pf_odom_pose_ = GetOdoPose;
		//if(!(++resample_count_ % 2))
		if(!(++resample_count_ % resample_interval_))
		{
			pf_update_resample(pf_,resample_upd);
			resampled = true;
		}
	

		set = pf_->sets + pf_->current_set;
		//printf("当前周期的粒子数目: %d\n", set->sample_count);
		
		if(resampled || force_publication)
		  {
		    // Read out the current hypotheses
		    //int total_count = 0;
		    int score_count = 0;
		    std::vector <amcl_hyp_t> hyps;
		    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
		    for(int hyp_count = 0;
			hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
		    {
		      
		      pf_vector_t pose_mean;
		      pf_matrix_t pose_cov;
		      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
		      {
			printf("Couldn't get stats on cluster %d", hyp_count);
			break;
		      }
		      hyps[hyp_count].weight = weight;
		      hyps[hyp_count].pf_pose_mean = pose_mean;
		      hyps[hyp_count].pf_pose_cov = pose_cov;

		      if(hyps[hyp_count].weight > max_weight)
		      {
			max_weight = hyps[hyp_count].weight;
			max_weight_hyp = hyp_count;
		      }
		    }

		if(first_located == false)
		
		for (int i = 0; i < set->sample_count; i++)
		{  
		pf_sample_t* sample = set->samples + i;
			
		{
		if(dis_among(hyps[max_weight_hyp].pf_pose_mean,sample->pose,0.10))
		    {
			score_count++;
		    }
		}
			
		}
			hyps[max_weight_hyp].score = (float)(score_count)/((float)(set->sample_count));
			locate_score = hyps[max_weight_hyp].score;
		if(max_weight > 0.0)
		{
///////////////////////////////////////最高得分的位置值赋值给全局变量LWG/////////////////////////////////////////

		cc_time=get_current_time_us();

		_amcl_pose.v[0] = hyps[max_weight_hyp].pf_pose_mean.v[0];//+ GetOdoPose_delta.v[0];//+BasePose_delta.v[0];
		_amcl_pose.v[1] = hyps[max_weight_hyp].pf_pose_mean.v[1];//+ GetOdoPose_delta.v[1];//+BasePose_delta.v[1];
		_amcl_pose.v[2] = hyps[max_weight_hyp].pf_pose_mean.v[2];//+ GetOdoPose_delta.v[2];//+BasePose_delta.v[2];
		}
		int j;
		j = MAP_GYWY(map_, _amcl_pose.v[1]);

		_amcl_rpose.x = _amcl_pose.v[0];
		_amcl_rpose.y = _amcl_pose.v[1];
		_amcl_rpose.theta = _amcl_pose.v[2];
		_amcl_rpose.timestamp = last_radar_data.timestamp;

		update_last_position(_amcl_rpose);
		//locate_pose.x = _amcl_pose.v[0];
		//locate_pose.y = MAP_WYGY(map_, map_->size_y-j);//_amcl_pose.v[1];
		//locate_pose.theta = -_amcl_pose.v[2];
		locate_pose.x = _amcl_pose.v[0];
		locate_pose.y = _amcl_pose.v[1];
		locate_pose.theta = _amcl_pose.v[2];
		locate_pose.timestamp = _amcl_rpose.timestamp;
		//printf("amcl locating pose x=%f,y=%f,theta=%f\n",locate_pose.x,locate_pose.y,locate_pose.theta);

		//latest_tf_valid_ = true;
    	}
		
		}////////if(resampled || force_publication)
	
}

void Amcl_gl::freeMapDependentMemory()
{
  map_loaded_success = false;
  if( map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
    //my_globalplanner.~GlobalPlanner();
  }
  if( pf_ != NULL ) {
    pf_free( pf_ );
    pf_ = NULL;
  }
  if( odom_ != NULL ) {
  delete odom_;
  odom_ = NULL;
  }
  if( laser_ != NULL ) {
  delete laser_;
  laser_ = NULL;
  }
  
}


bool	Amcl_gl::Initial_Location_Thread(bool bin)////机器人初始定位线程函数
{
	int index_j;
	float particle_radius = 0.2;
	if(first_located_invoked == true)
	{/////////////定位步骤激活

		if (locate_score>0.90 || bin)
					{
					pf_vector_t pf_init_pose_mean = pf_vector_zero();
					
					pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
					/////////////////////////////使用得到的位置重新初始化粒子群
					if (locate_step<3 && !bin)
					{////启动二次重定位步骤
					
					usleep(30000);
					
					locate_step++;
					printf("\n========================\n启动重--%d--撒粒子步骤\n=========================\n", locate_step);
					printf("初始定位完成\n最大置信度的位姿点为x%f>>y%f>>a%f>>>score =%f\n", _amcl_pose.v[0],
						_amcl_pose.v[1], _amcl_pose.v[2], locate_score);
					flush_odom_data();
					pose2d = _amcl_rpose;
					flush_odom_data_raw();
					pose2d_raw = _amcl_rpose;
					usleep(200000);
					pf_init_pose_mean = _amcl_pose;
					pf_init_pose_cov.m[0][0] = particle_radius;
					pf_init_pose_cov.m[1][1] = particle_radius;
					pf_init_pose_cov.m[2][2] = 0.1;
					pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
					pf_init_ = false;
					
					}
					else
					{
					/////////////////////////////////////////////////////////////////////////
					printf("初始定位完成\n最大置信度的位姿点为x%f>>y%f>>a%f>>>score =%f\n", _amcl_pose.v[0],
						_amcl_pose.v[1], _amcl_pose.v[2]/M_PI*180, locate_score);
					
					flush_odom_data();
					pose2d = _amcl_rpose;
										
					first_located_invoked = false;
					first_located = true;
					if(bin)
					usleep(15000);
					if (bin)
					printf("+++++++++++++++++++++++++++++++++机器人强制定位完成\n");
					else
					printf("==========================================定位完成\n");
					}
					usleep(15000);
					}

	}


}


bool Amcl_gl::jh_initial_pf()
{
	printf("成功加载地图，执行粒子初始化\n");
	particle_filter_initialize();
	////////////////////////////////////////////
	/////////////////////////////////////////////设定激光的安装位置变量
	pf_vector_t laser_pose_v;
	laser_pose_v.v[0] = para_LASER_2_BASE_X;
	laser_pose_v.v[1] = para_LASER_2_BASE_Y;///laser_pose_v.v[1] = LASER_2_BASE_DIS;
	// laser mounting angle gets computed later -> set to 0 here!
	laser_pose_v.v[2] = para_LASER_2_BASE_A;
	printf("激光安装方向%f\n",para_LASER_2_BASE_A);
	laser_->SetLaserPose(laser_pose_v);
	return true;
}

void  Amcl_gl::main_particle_run()//////主粒子滤波运行
{
	
/////////////////////////////////////////////开机执行初始化一次，初始化粒子滤波器，odom，激光传感器
	bool force_located = false;////超时（LOCATE_TIME_EXCEED）强制定位完成
	bool b_ini_located = false;////初始定位初始化变量
	char force_update;/////定时强制触发粒子定位

	/* pf_vector_t pf_init_pose_mean = pf_vector_zero();
	  pf_init_pose_mean.v[0] = -0.5;
	  pf_init_pose_mean.v[1] = 0.0;
	  pf_init_pose_mean.v[2] = 0.2;
	  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
	  pf_init_pose_cov.m[0][0] = 0.5;
	  pf_init_pose_cov.m[1][1] = 0.5;
	  pf_init_pose_cov.m[2][2] = 0.2;
	  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);	 
*/
	long ini_time = get_current_time_us();
	long real_time = ini_time;
	double delta_t;
	int LOCATE_TIME_EXCEED = 70 * 1000;
	locate_step = 1;

	while (first_located_invoked)
	{
//////////////////////////////////////////////////////////////机器人初始定位超时判断
	
		if(b_ini_located == false)
		{
			b_ini_located = true;
			force_located = false;
			ini_time=get_current_time_us();
			delta_t = 0;////清零delta_t
		}
		real_time = get_current_time_us();
		delta_t = double(real_time-ini_time)/1000;
		if(delta_t > LOCATE_TIME_EXCEED) 
		{force_located = true;////超时强制定位完成
		delta_t = 0;//
		}
		

	
//////////////////////////////////////////////////////////////////
	force_update = 1;/////这里采用的全局定位，直接开启强制粒子更新过程

	particle_filter_run(force_update);////////启动粒子滤波器
	Initial_Location_Thread(force_located);////机器人初始定位线程函数
	force_update = 0;
	usleep(20000);
	}
	printf("定位线程运行结束\n");
}


bool Amcl_gl::dis_among(pf_vector_t pos1, pf_vector_t pos2, double dis)/////两个空间点之间的距离
{
	double dis_ret;
	double dis_x = pos1.v[0] - pos2.v[0];
	if (fabs(dis_x)>dis) return false;
	double dis_y = pos1.v[1] - pos2.v[1];
	if (fabs(dis_y)>dis) return false;

	double v02 = (dis_x)*(dis_x);
	double v12 = (dis_y)*(dis_y);
	dis_ret = sqrt(v02 + v12);
	if (dis_ret>dis)
		return false;
	else
		return true;
}

bool Amcl_gl::start_global_locating()
{
	//printf("开始全局定位线程11111111111\n");
	first_located = false;
	usleep(500000);
	freeMapDependentMemory();
	laser_recv_cnt = 10;
	map_ = do_map_open();
	jh_initial_pf();
	first_located_invoked = true;
	
	pthread_create(&handle_global_locating, nullptr, locating_thread, this);
	//main_particle_run();
	return true;
}
 void * Amcl_gl::locating_thread(void* para)//////
{
	//printf("开始全局定位线程22222222222\n");
    Amcl_gl *ptr = (Amcl_gl*)para;
    ptr->main_particle_run();
    return nullptr;
}

