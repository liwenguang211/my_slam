#ifndef AMCL_GL_H
#define AMCL_GL_H
#include "amcl_odom.h"
#include "amcl_laser.h"
#include "map.h"
#include "pf.h"
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
using namespace amcl;
typedef unsigned char byte;

#define fuzzy_1 0.001
#define fuzzy_mini 0.01
//宏定义
////////////////////////////////////////////////////////////////////////
pf_vector_t uniformPoseGenerator(void* arg);///////初始均布粒子
class Amcl_gl : public MsgQueueListener {
public:
	//Amcl_gl() ;
	Amcl_gl(MsgQueue *queue, int radar_chan,int odom_chan);
    virtual void recieve_message(const int channel, char *buf, const int size);
	~Amcl_gl();
public:
	int min_particles_;////允许的粒子数量的最小值，默认100(500)
	int max_particles_;////允许的例子数量的最大值，默认5000
	float kld_err;    //真实分布和估计分布之间的最大误差，默认0.01(0.05)
	float kld_z;  //上标准分位数（1-p），其中p是估计分布上误差小于kld_err的概率，默认0.99
	float d_thresh_;//////粒子滤波触发条件1(0.2)
	float a_thresh_;//////粒子滤波触发条件2(0.5)
	int resample_interval_;////重采样间隔(2)
	float recovery_alpha_slow; //慢速的平均权重滤波的指数衰减频率，用作决定什么时候通过增加随机位姿来recover，默认0（disable），可能0.001是一个不错的值
	float recovery_alpha_fast;  //快速的平均权重滤波的指数衰减频率，用作决定什么时候通过增加随机位姿来recover，默认0（disable），可能0.1是个不错的值
	float save_pose_rate;  //存储上一次估计的位姿和协方差到参数服务器的最大速率。被保存的位姿将会用在连续的运动上来初始化滤波器。-1.0失能。(0.5)


	/////激光模型参数
	float laser_min_range;  //被考虑的最小扫描范围；参数设置为-1.0时，将会使用激光上报的最小扫描范围
	float laser_max_range;  //被考虑的最大扫描范围；参数设置为-1.0时，将会使用激光上报的最大扫描范围
	float laser_max_beams;  //更新滤波器时，每次扫描中多少个等间距的光束被使用（减小计算量，测距扫描中相邻波束往往不是独立的可以减小噪声影响，太小也会造成信息量少定位不准）
	float laser_z_hit; //(0.5)模型的z_hit部分的混合权值，默认0.95(混合权重1.具有局部测量噪声的正确范围--以测量距离近似真实距离为均值，其后laser_sigma_hit为标准偏差的高斯分布的权重)
	float laser_z_short; //(0.05)模型的z_short部分的混合权值，默认0.1（混合权重2.意外对象权重（类似于一元指数关于y轴对称0～测量距离（非最大距离）的部分：
	float laser_z_max; //0.05模型的z_max部分的混合权值，默认0.05（混合权重3.测量失败权重（最大距离时为1，其余为0）
	float laser_z_rand; //0.5模型的z_rand部分的混合权值，默认0.05（混合权重4.随机测量权重--均匀分布（1平均分布到0～最大测量范围）
	float laser_sigma_hit; //0.2被用在模型的z_hit部分的高斯模型的标准差，默认0.2m
	float laser_lambda_short; //0.1模型z_short部分的指数衰减参数，默认0.1（根据ηλe^(-λz)，λ越大随距离增大意外对象概率衰减越快）
	float laser_likelihood_max_dist; //2.0地图上做障碍物膨胀的最大距离，用作likelihood_field模型
	int laser_model_type; //LASER_MODEL_LIKELIHOOD_FIELD//likelihood_field模型使用，可以是beam, likehood_field, likehood_field_prob
	bool do_beamskip_; //////false
	float beam_skip_distance_; ///0.5
	float beam_skip_threshold_; ///0.3
	//里程计模型参数
	odom_model_t odom_model_type; //ODOM_MODEL_DIFF//diff模型使用，可以是"diff", "omni", "diff-corrected", "omni-corrected",后面两 
	float odom_alpha1; //0.2指定由机器人运动部分的旋转分量估计的里程计旋转的期望噪声，默认0.2
	float odom_alpha2; //0.2制定由机器人运动部分的平移分量估计的里程计旋转的期望噪声，默认0.2
	float odom_alpha3; //0.8指定由机器人运动部分的平移分量估计的里程计平移的期望噪声，默认0.2
	float odom_alpha4; //0.2指定由机器人运动部分的旋转分量估计的里程计平移的期望噪声，默认0.2
	float odom_alpha5; //0.1平移相关的噪声参数（仅用于模型是“omni”的情况--wiki官网的注释）

	////////////////////////////////
	int resample_count_;///////重采样计数
	float ini_ang=60;
	float end_ang = 480;
	float ang_step = 0.5;
	float range_max = 20;
	float range_min = 0.2;
	SimpleGridMap *gridmap;
	bool para_initial();
	bool set_laser_para(Position radarpos,float radar_ini_ang,float radar_end_ang,float radar_step,float radar_max_range,float radar_min_range);
public:
	int particle_filter_initialize();///////初始化粒子滤波器
	map_t* do_map_open(); ////////地图数据导入
	void freeMapDependentMemory();////释放地图，odo，laser,particl所占用的内存空间
	
	bool dis_among(pf_vector_t pos1, pf_vector_t pos2, double dis);/////两个空间点之间的距离
	bool jh_initial_pf();
	bool particle_filter_run(char fup);////////启动粒子滤波器
	bool	Initial_Location_Thread(bool bin);////机器人初始定位线程函数
	void main_particle_run();//////主粒子滤波运行
	static void * locating_thread(void* ptr);//////
	bool start_global_locating();
	pthread_t handle_global_locating;
	float para_LASER_2_BASE_X;
	float para_LASER_2_BASE_Y;
	float para_LASER_2_BASE_A;
	float init_pos_x;//////通过界面传递给amcl算法的位姿
	float init_pos_y;
	float init_pos_theta;
	PointCloudData last_radar_data;
	Position radar_pos;
	int laser_recv_cnt = 10;
	void set_init_pos(float x,float y,float theta);
public:
	////////////////////////全局定位状态
	bool first_located_invoked;
	bool first_located;
	float locate_score;
	Position locate_pose;
	char locate_step;
	pf_vector_t _amcl_pose;
	Position _amcl_rpose;
	long last_time_pose;////上一步里程计的时间
	bool map_loaded_success;////
	FILE *map_raw_data_file;////未处理地图数据文件
	int record_order;
	bool first_record;
	Position last_record_pos;
	bool begin_mapping;
	void _begin_map_create();
	void _end_map_create();
	void _recor_data2map();
	void _recor_data2xm();
	void update_last_position(Position &pose);
	Position expolate_current_position_tim(long timestamp);
	Position expolate_current_position();
	void add_odom_data(Position &pose_change);
    	void update_odom_list_timestamp(long timestamp_us);
    	void flush_odom_data();
	

	
    	void add_odom_data_raw(Position &pose_change);
    	void update_odom_list_timestamp_raw(long timestamp_us);
    	void flush_odom_data_raw();
	Position expolate_current_position_tim_raw(long timestamp);
	Position expolate_current_position_raw();
private:
	pf_t *pf_;/////粒子滤波器
	bool pf_init_;
	MsgQueue *queue_a;
	//bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...
	AMCLOdom* odom_;
	AMCLLaser* laser_;
	map_t* map_;/////地图数据	
	pf_vector_t pf_odom_pose_;////last odo data
	bool lasers_update_;
	int radar_c;
	int odom_c;
	pthread_mutex_t listen_pose_mutex;
	Position radar_position;

	int pose_level = 1;
	Position pose2d;
	pthread_mutex_t mutex_pose;
	std::list<Position> odom_list;
    	pthread_mutex_t mutex_odom_list;
    
	Position pose2d_raw;
	pthread_mutex_t mutex_pose_raw;
	std::list<Position> odom_list_raw;
    	pthread_mutex_t mutex_odom_list_raw;

	int robot_stop_cnt = 0;///
};
/////////////////////////////////////////////////////////////////////////////////






#endif
