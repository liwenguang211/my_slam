#ifndef CURVE_THREAD_H
#define CURVE_THREAD_H

/* Author: LWG */

/**


*/
#include <vector>
#include "common/pose.h"
#include "common/time.h"
#include "common/speed_data.h"
using namespace std;
#include "msg_queue.h"
#include "move.h"
#define ANGLE_ADD_MAX	(0.3) //////角度偏差积分最大值
#define NONSENSEVAL		(65535)//////轨迹点索引无效的值
/////////////////////////////////////////////李雅普诺夫函数参数
#define LYP_KX			(0.00)
#define LYP_KY			(0.00)
#define LYP_KPHY		(0.00)
#define LYP_KPHY1		(0.05)
#define LYP_KP_DEC		(0.5)////减速阶段的旋转比例常数
#define LYP_TTP			(1.0)
#define THETA_TORLERANCE	(0.20)
#define FUZZY_MINI		(0.007)
typedef enum  
{
	MODE_FREE=47,
	MODE_CREATEMAP,
	MODE_MAP_UPING,
	MODE_LOCATING,
	MODE_FIRST_LOCATING,
	MODE_MANUAL,
	MODE_AUTO,
	MODE_PTP,
	MODE_CURVE_MOVE,
} MODE_OPERATION;
typedef enum  
{
	MOTION_NULL=67,
	MOTION_COMPLETE,
	MOTION_ACC,
	MOTION_DEC,
	MOTION_EVEN,
	MOTION_ACCDEC,
	MOTION_ACC_LOWSP,
} MOTION_TYPE;///////////////轨迹运动类型0516
typedef enum  
{
	TRAJ_NONE=77,////无效轨迹类型
	TRAJ_ROTATE,////运动为旋转
	TRAJ_LINE,////运动为直线
	TRAJ_START,////段类型为起始段
	TRAJ_END,/////段类型为结束段
	TRAJ_MIDDLE,/////段类型为过渡段
	TRAJ_COMPLETE,////段类型为完整段
  TRAJ_EXCUT_TASK,////段类型为任务段
	TRAJ_ISPTP,//////轨迹类型为PTP
	TRAJ_ISCURVE,////轨迹类型为曲线
	TRAJ_ISENDDING,////最后一段轨迹
  TRAJ_ISSLIDING,////最后一段轨迹
	TRAJ_FINISHED,//////表示后续没有轨迹
	TRAJ_ISFREE,//////轨迹类型为PTP
	//TRAJ_NONE,
} TRAJTORY_TYPE;///////////////轨迹运动类型
typedef enum  
{
	TRAJ_RUN_STEPACC=97,////运行在轨迹加速阶段
	TRAJ_RUN_STEPEVEN,/////运行在轨迹的匀速阶段
	TRAJ_RUN_STEPDEC,/////运行在轨迹的减速阶段
} TRAJTORY_RUN_STEP;///////////////轨迹运行阶段
typedef enum  
{
	TRAJ_LOOKAHEAD_OK=107,
	TRAJ_LKAH_PATHPOINT_ERROR,
	TRAJ_NEED_NEXTP_Y,////需要规划下一个路径点
	TRAJ_NEED_NEXTP_N,////不需要规划下一个路径点
	TRAJ_POINT_TYPE_LM,////路径点LM类型
	TRAJ_POINT_TYPE_CP,////路径点CP类型
	TRAJ_POINT_TYPE_HOME,////路径点HOME类型
	TRAJ_TYPE_BEZI,////轨迹类型贝塞尔类型
	TRAJ_TYPE_LINE,////轨迹类型直线类型
} TRAJTORY_LOOKAHEAD;///////////////轨迹前瞻
typedef enum  
{
	ROBOT_TASK_STS_NONE = 157,
	ROBOT_TASK_STS_WAITING,
	ROBOT_TASK_STS_RUNNING,
	ROBOT_TASK_STS_SUSPENDING,
	ROBOT_TASK_STS_COMPLETE,
	ROBOT_TASK_STS_FAILED,
	ROBOT_TASK_STS_CANCELED,
} ROBOT_TASK_STS;///////////////机器人task状态
typedef struct 
{
	Position CURVE_TARGET_POSE;///////期望运行到的位姿点
	Position CURVE_INITIAL_POSE;///////运行的初始点位姿
	Position CURVE_CONTROLP1;///////控制点1
	Position CURVE_CONTROLP2;///////控制点2
	Position ST_LKAHP;///////起始look_ahead_dis的点
	Position ED_LKAHP;///////终止look_ahead_dis的点
	int ST_LKAHP_u;/////对应起始look_ahead_dis的点的u
	int ED_LKAHP_u;///对应终止look_ahead_dis的点的u
	int stat_index;/////起始点索引
	int end_index;//////结束点索引
	int curve_index;/////曲线索引
	int total_u_num;/////曲线的总u计数
	int slow_u;////开始减速的索引位置
	int init_u;/////机器人第一次启动时，对应的最近点的u
	int init_next_u;/////机器人第一次启动时，对应的最近点的next u
	double length;

	//int traj_type;////轨迹类型,起始段，终止段，过渡段，完整段
	//int PTP_OR_CURVE;/////PTP还是贝塞尔轨迹
}T_CURVE_P;///////曲线结构体
typedef struct 
{
	T_CURVE_P tarjp;
	int traj_type;////轨迹类型,起始段，终止段，过渡段，完整段
	int trisend;//////该轨迹段是结束标识
	int PTP_OR_CURVE;/////PTP还是贝塞尔轨迹
	char isinverse;//////轨迹运动反向
	float vel_line;////该段轨迹的线速度
	float vel_ang;///该段轨迹的角速度
}T_TRAJ_;///////轨迹结构体
typedef struct 
{
	Position PTP_TARGET_POSE;///////期望运行到的PTP位姿点
	Position PTP_INITIAL_POSE;///////PTP运行的初始点位姿
	Position PTP_TARGET_DIR;///////PTP运行的方向矢量
	Position PTP_LAST_POSE;///////PTP运行的上一步点位姿，用于计算vref，wref
}T_TRAJ_PARA;
typedef struct 
{
	Position T_INIP;////临时轨迹起始点
	Position T_ENDP;////临时轨迹终止点
	Position T_NEXTP;////临时轨迹下一点
	int T_INIPID;////临时轨迹起始点索引
	int T_ENDPID;////临时轨迹终止点索引
	int T_NEXTPID;////临时轨迹下一点索引
	bool T_INIPRDY;////临时轨迹起始点READY
	bool T_ENDPRDY;////临时轨迹终止点READY
	bool T_NEXTPRDY;////临时轨迹下一点READY
	int traj_index;////轨迹索引号
}T_REALTIMEDATA;///////实时轨迹参数结构体
class curve_run: public MsgQueueListener {
public:
    curve_run(MsgQueue *queue,int move_chan);
    curve_run();
    ~curve_run();
    virtual void recieve_message(const int channel, char *buf, const int size);
private:
	MsgQueue *queue;
	int chan_move;
	bool lift_up_finished ;
	bool lift_down_finished;
	void send_lift_msg(int dir);
	char move_flag = 1;//////测试顶升用 0-无动作，1-上升，2-下降
public:
	pthread_t manu_run_thread;
	pthread_t curve_run_thread;
    Position target_pos;
    T_REALTIMEDATA	running_data;/////轨迹运行的实时数据
    int i_step;
    T_TRAJ_ CURBEZIER;
    vector <T_TRAJ_> myspace_traj;////从下发指令推算出的轨迹
    int set_path(vector <Position> pathpoints,vector <float> vel_points,vector <char> inverse_s,char do_act);
    int initial_path(vector <Position> pathpoints,vector <float> vel_points,vector <char> inverse_t,char do_act);
    Position cal_CZ(Position pos0,Position pos1,Position pos2,int &dir);////计算垂足点
    Position curp2curve(Position pcur,T_CURVE_P &curtraj,int curvetype);
    ////计算由当前点到曲线上距离最近的点
    Position find_initial_point( Position pcur,int &init_i, T_CURVE_P &curtraj,int curvetype);/////求解bezier曲线上的起始点坐标
    Position BezierPoint(double t, T_CURVE_P curtraj);/////求解bezier曲线上的点
    double sqsum(Position pos0,Position pos1);
    double getBezier_length( T_CURVE_P &curtraj);////求解bezier曲线的长度
    bool dis_between(Position pos1,Position pos2,double &dis);/////两个空间点之间的距离
    bool traj_par_init();
    bool dis_among(Position pos1,Position pos2,double dis);/////两个空间点之间的距离
    static void * curve_move_thread_(void * param);
    static void * manual_run_thread_(void * param);
    double angle_betw(Position pos1,Position pos2,Position pos3);/////三点间的夹角
    void manual_run();
    double  jh_sign(double in);
    bool dvect_between(Position pos1,Position pos2,Position &dvector,double &l_tlen);/////两个空间点之间的单位方向矢量
    void exchangetraj();/////交换轨迹参数
    bool jh_vel_test(double &line_vel,double &angle_vel);
    void CURVE_THREAD_RUN(long &pre_time,int &t_cur,int &t_acc,int &t_dec,int &t_even,double &l_even,double &l_acc,double &l_dec,double &l_total,char &motion_type,char &runstep);
    void loadtrajpar();/////加载轨迹参数
    Position cal_deltap_(char type,float dt,int &cur_t,int t_acc,int t_dec,int t_even,double l_even,double l_acc,double l_dec,double l_total,char motion_type,double &vrf,double &wrf,int &needsts);////由路径规划参数计算deltapos，vr和wr用于机器人的驱动
    T_TRAJ_ ptp2curvetraj(Position ps,Position pe,float speed,char inverse_s,int traj_typ);/////点到点组合成轨迹参数
    T_TRAJ_ ptp2traj(Position ps,Position pe,float speed,char inverse_t,int traj_typ);/////点到点组合成轨迹参数
    float  msine(float nor_time);
    void nexttraj();/////切换到下一段轨迹
    bool  JH_LEN2T(float len,char type,int &t_acc,int &t_dec,int &t_even,double &l_even,double &l_acc,double &l_dec,double &l_total,char &motion_type);//由运行路程计算该轨迹的运动参数
    bool mode_auto2manual();/////从自动模式切换至手动模式的处理
    bool cancel_path();//////轨迹取消
    bool stop_path();/////暂停轨迹
    bool continue_path();/////继续轨迹
    int TRAJ_RUNNING_STEP;
    int task_status;
    Position realpose_map;
    double cmd_angle_vel;
    double cmd_line_vel;
    int running_time;
    int line_cnt;
    int rotate_cnt;
    bool dir_f=false;
    bool dir_b=false;
    bool dir_l=false;
    bool dir_r=false;
    bool TRAJDONE;
  private:
    char mode_manual_auto = MODE_MANUAL;
    int target_reached_count;
    bool  LOOKAHEADFNISED = false;
    float nav_vel_angle_p = 0.7;
    float look_ahead_dis = 0.1;
    float nav_slow_dis= 0.27;
    float pass_allow_dis = 0.02;
    float nav_dec_line = 0.3;
    float nav_acc_line = 0.3;
    float nav_vel_line = 1.0;

    float nav_dec_angle = 0.3;
    float nav_acc_angle = 0.3;
    float nav_vel_angle = 0.5;
    float min_angle_vel = 0.1;
    float min_line_vel = 0.05;

    float para_WHEEL_DISTANCE = 0.6;
    float vel_angle_recal = 2*nav_vel_line/para_WHEEL_DISTANCE*0.25;
    float dis_line_dec = 0.5*nav_vel_line*nav_vel_line/nav_dec_line+nav_slow_dis;
    float allow_angle = 0.01;
    float allow_distance = 0.005;/////容许的位移偏差
    float vel_ratio = 1.0;
    float targetangle;
    double auto_run_lvel = 0.0;
    double auto_run_rvel = 0.0;
    float vel_start = 0.0;
    float manual_run_lvel = 0.0;
    float manual_run_rvel = 0.0;
    double line_vel,angle_vel;
    T_TRAJ_PARA Traj;////////////轨迹规划参数
};

#endif
