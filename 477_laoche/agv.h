#ifndef AGV_INSTANCE_H
#define AGV_INSTANCE_H

#include <stdio.h>
#include <pthread.h>
#include <list>

#include "common/simple_grid_map.h"
#include "common/pose.h"
#include "common/time.h"
#include "common/capture_point.h"
#include "msg_queue.h"
#include "radar.h"
#include "nav.h"
#include "carto.h"
#include "odometer.h"
#include "device.h"
#include "imu.h"
#include "storage.h"
#include "pose_correct.h"
#include "amcl_gl.h"
#include "low_resolution.h"
#include "pallet_recgnize.h"
#include "curve_run_fn.h"
#include "motor.h"
#define SYSTEM_STATE_OFFLINE    0
#define SYSTEM_STATE_FREE       1
#define SYSTEM_STATE_MAPPING    2
#define SYSTEM_STATE_LOCATING   3
#define SYSTEM_STATE_NAVIGATING 4
#define SYSTEM_STATE_PAUSE      5
#define SYSTEM_STATE_OPTIMISM   6
#define SYSTEM_STATE_INIT_POSE  7
#define SYSTEM_STATE_OPERATION  8
#define SYSTEM_STATE_MAPPING_LANDMARK  9

class MapSharedMemory {
public:
    MapSharedMemory(MapInfo &info) : map(new SimpleGridMap(info)) {
        pthread_mutex_init(&mutex, nullptr);
    }
    MapSharedMemory(SimpleGridMap &m) : map(new SimpleGridMap(m)) {
        pthread_mutex_init(&mutex, nullptr);
    }
    MapSharedMemory(char * data_buf, int buf_size) : map(new SimpleGridMap(data_buf, buf_size)) {
        pthread_mutex_init(&mutex, nullptr);
    }
    ~MapSharedMemory() {
        pthread_mutex_destroy(&mutex);
        delete map;
    }
    void increase(int i) {
        pthread_mutex_lock(&mutex);
        count += i;
        pthread_mutex_unlock(&mutex);
    }
    bool free(int i) {
        bool ret = false;
        pthread_mutex_lock(&mutex);
        count -= i;
        ret = (count < 1);
        pthread_mutex_unlock(&mutex);
        return ret;
    }
    pthread_mutex_t mutex;
    SimpleGridMap *map;
    int count = 0;
};

class MapSharedPointer {
public:
    MapSharedPointer() {};
    MapSharedPointer(MapSharedMemory * p_mem):mem(p_mem) {
        if(mem != nullptr) {
            mem->increase(1);
        }
    };
    MapSharedPointer(MapSharedPointer &ptr) : MapSharedPointer(ptr.mem) {};
    MapSharedPointer& operator=(const MapSharedPointer& ptr) {
        if(&ptr != this) {
            mem = ptr.mem;
            if(mem != nullptr) {
                mem->increase(1);
            }
        }
        return *this;
    };
    MapSharedPointer& operator=(MapSharedMemory * p_mem) {
        if(mem != nullptr) {
            if(mem->free(1)) {
                delete mem;
            }
        }
        mem = p_mem;
        if(mem != nullptr) {
            mem->increase(1);
        }
        return *this;
    };

    ~MapSharedPointer() {
        if(mem != nullptr) {
            if(mem->free(1)) {
                delete mem;
            }
        }
    };

    bool is_nullptr() {
        return mem == nullptr;
    }
    SimpleGridMap *get() {
        return mem->map;
    }
    SimpleGridMap &operator*() {
        return *(mem->map);
    };
    SimpleGridMap *operator->() {
        return mem->map;
    }

private:
    MapSharedMemory *mem = nullptr;
};

typedef struct {
    float pose[3];
    std::vector<double> points;
} LandmarkRadarData;

class AGVInstance : public MsgQueueListener {

public:
    AGVInstance() {initialize();};
    AGVInstance(const AGVInstance &) = delete;
    AGVInstance& operator=(const AGVInstance&) = delete;
    ~AGVInstance() {shutdown();};

    int get_system_state();
	char * My_strsub_i2e(char * src,int start_i,int len)
{
    	assert(src != NULL);
	int total_length = start_i+len;//
	int length = total_length - start_i;
	int real_length = ((length) >= 0 ? length : total_length)+1;
	char *tmp;
	//char *ret;
	if (NULL == (tmp=(char*) malloc(real_length * sizeof(char)))) {
		return NULL;
	 }
	strncpy(tmp, src+start_i, real_length - 1);
	    tmp[real_length - 1] = '\0';
	return tmp;	
}
    Position get_last_position();
    Position expolate_current_position();

    Position expolate_current_position_tim(long timestamp);/////////lwg20201210
    void update_last_position(Position &pose);
    void update_last_position(Position &pose, int my_level, int change_level_to);

    Position get_raw_odom_position();
    bool set_raw_odom_position(Position &pose);
	double obtain_imu_angle();
    MapSharedPointer get_current_map() {return p_map;};
    void load_grid_map(SimpleGridMap &map);
    void load_grid_map(char *buf, int size);

    MsgQueue * get_message_queue() {return message_queue;};

    virtual void recieve_message(const int channel, char *buf, const int size);

    bool start_mapping();
    bool stop_mapping();
    bool start_mapping_landmark();
    bool stop_mapping_landmark();

    bool start_navigating(CapRoute *r);
    bool start_navigating(RouteChain c);
    bool pause_navigating();
    bool stop_navigating();
	bool pose_initilized = false;
	int exec_chain;
	int exec_route;
	int exec_cap;
	//mcu状态
	int mcu_input;
	int mcu_output;
	int mcu_battery_level;
	int mcu_speed;
	int mcu_omega;

    void start_global_locating();
    // void start_global_locating(double x, double y);

    PointCloudData get_last_radar_data();

    std::list<CapPoint> caps;
    std::list<CapRoute> routes;
    std::list<RouteChain> chains;
	
    std::list<Position> masks;
	double mask_radius;
	bool mask_switch;
	bool collision_switch=true;
	bool nav_thread_running = false;
	
	
	
	ROUTE r[ROUTE_SLOT_NUM];
	int map_id;
	char map_name[200];
	storage *st;
	int collision_level=0;
	int colli_cnt1=0;
	int colli_cnt2=0;
	int colli_cnt3=0;
	
	
	int default_locate_map_id;
	bool loadPoseFromserver();/////lwg20201218将机器人位姿从文件中加载	
	bool loadPose(char* src);/////clm20201228将机器人位姿从UI中加载	
	
	int pallet_recognize(float len,float width,float angle_sech,int clc_cnt,Position & result_pose);////托盘位姿识别
	int legs_recognize(float len,float width,float angle_sech,int clc_cnt,Position & leg1_pose,Position & leg2_pose);////托盘退部识别
	int start_path(vector <Position> pathpoints,vector <float> vel_points,vector <char> inverset,char doact);////ret -1 failed; 1 success;doact(0-no action,1-up,2-down)
	bool cancel_path();
	bool stop_path();
	bool continue_path();
private:
    void initialize();
    void shutdown();
    void registe_listener();

    int state = SYSTEM_STATE_OFFLINE;
    pthread_mutex_t mutex_state;

    MsgQueue * message_queue = nullptr;

    Position pose2d;
    int pose_level = 1;
    pthread_mutex_t mutex_pose;

    MapSharedPointer p_map;
	SimpleGridMap *low_reso_map;//////用于定位的低分辨率地图
    pthread_mutex_t mutex_map;

    PointCloudData last_radar_data;
    pthread_mutex_t mutex_radar_data;

    std::list<Position> odom_list;
    pthread_mutex_t mutex_odom_list;
    void add_odom_data(Position &pose_change);
    void update_odom_list_timestamp(long timestamp_us);
    void flush_odom_data();

    pthread_mutex_t mutex_raw_odom;
    Position raw_odom;
    void add_raw_odom_data(Position &pose_change);

    static void * random_rotate_thread_function(void * param);
    pthread_t random_rotate_thread;
    bool do_random_rotate = false;

    void do_global_locating();
    static void * global_locate_thread_function(void * param);
    pthread_t global_locate_thread;

    Position do_scan_matching(PointCloudData &data, Position &pose, char * config_item);
    //void do_scan_matching(PointCloudData &data);
	void do_scan_matching(PointCloudData &data,Position &pos_ret ,bool &is_correct);///////////////////20201210LWG
	Position cur_pos;////本次的里程计数据
	Position last_pos;////上次里程计的数据
	bool scan_init;///////////////lwglwg20201210
	int insert_index;////////////lwg20201210
	int on_scan_matching_cnt_;/////开机后的scan-matching的计数lwg20201210
	raw_pos_for_correct raw_pose;////////////lwg20201210
	
	bool init_locate();//////lwg20201218初始化启动全局定位
	bool savePoseToServer();/////lwg20201218将机器人实时位姿写入到文件
//	bool loadPoseFromserver();/////lwg20201218将机器人位姿从文件中加载
	int record_pose_cnt;/////用于记录机器人当前位姿的计数，大于100，则将位姿数据写入到文件中
	int scan_matching_cnt;/////用于定时触发scan-matching
	static void * global_amcl_locate_thread_function(void * param);
    	pthread_t global_amcl_locate_thread;
	void do_amcl_global_locating();
	long last_time_pose;////上一步里程计的时间
	int robot_stop_cnt = 0;///机器人停止计数
	bool home_tuning;/////原地转向标识
	Position pose_from_save_file;
	Amcl_gl * amcl_global_locate;
	Pallet_recgnize * pallet_pose_recgnize;
    bool scan_matching_pause = false;
    bool doing_scan_matching = false;

	static void * pallet_recognize_thread_function(void * param);  ///////测试托盘识别使用
    	pthread_t pallet_recognize_thread;///////测试托盘识别使用
	void do_pallet_recognize();///////测试托盘识别使用


    static void * painting_map_thread_function(void * param);
    pthread_t painting_map_thread;
    bool painting_map_thread_running = false;
    std::list<LandmarkRadarData> landmark_radar_datas;
    int landmark_radar_data_insert_count = 0;
    void handle_landmark_data_on_painting(PointCloudData &data);

    // static void * navigating_thread_function(void * param);
    // pthread_t navigating_thread;
    // bool navigating_thread_running = false;
    // bool navigating_thread_pause = false;

	
	Position radar_position;

	IMUModule *imu = nullptr;

    DevicesModule * devs = nullptr;
    RadarModule * radar = nullptr;
    Odometer *odom = nullptr;
    CartoModule * carto = nullptr;
    NavModule * nav = nullptr;
	curve_run *my_runcurve = nullptr;
    //SlamKarto * my_karto = nullptr;
	Motor *motor =nullptr;
};

void init_global_agv_instance();
AGVInstance * get_global_agv_instance();
storage *get_global_storage();
#endif
