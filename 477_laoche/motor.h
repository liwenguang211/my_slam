#ifndef MOTOR_H_
#define MOTOR_H_

#include "common/canner.h"
#include "common/speed_data.h"
#include <vector>
#include "common/sensor_data.h"
#include "common/time.h"
#include "msg_queue.h"
#include "move.h"

typedef unsigned char u8;
typedef unsigned short int u16;
typedef unsigned int u32;



/**
 * @brief  Motor structure definition
 */
typedef struct
{
	const u8 NodeID; /* 站ID */

	volatile int ActualPosition; /* 编码器值 */
	volatile u16 StatusWord;	 /* 状态字 */

	volatile u8 EnableStatus; /* 使能状态 */
	volatile u8 Fault;		  /* 故障状态 */

	volatile int SpeedRatio; /* 速度下发数字量 */

	volatile int ExpectPositionRatio; /* 期望目标位置 编码器值 */

	long timeStamp; /*!< 单位：ms */

} MotorTypedef;

typedef struct {
	int ActualPosition_L; /* 举升电机编码器值 */
	int ActualPosition_T; /* 旋转电机编码器值 */

	float Height; /* 高度 mm (0-40mm) */
	float Angle; /* 角度 °	顺时针为正 */

} LiftMachineTypedef;



class Motor : public MsgQueueListener
{
public:
	Motor(MsgQueue *queue, int motor_chan, int wheel_chan, int move_chan);
	~Motor();
	void MotorDataAnalyze(MotorTypedef *thisMotor, u8 *data);
	void MotorRun(Speed_data data);
	void MotorRun(double v,double omega);
	void MotorStop(void);
	void motorPositionControl(MotorTypedef thisMotor);
	void motorSpeedControl(MotorTypedef thisMotor);
	void motorSetPositionMode(MotorTypedef thisMotor);
	void motorSetSpeedMode(MotorTypedef thisMotor); 
	void SwitchPDO(MotorTypedef thisMotor);
	void handle_obstacle_avoidance(Speed_data data);
	virtual void recieve_message(const int channel,char *buf, const int size);
	void Lift_MotorRise(void);
	void Lift_MotorFall(void);
	void Turn_MotorRun(float expect_angle, float omega);
	void Turn_MotorStop(void);
	void LT_Motor_getZeroPose(void);
	void LT_Motor_calcCurrentPose(void);
	void Lift_Motor_setExpectpose(float expect_H, float expect_A);
	bool LiftMachine_TaskComplete(void);
	u8 LiftMachine_gotoZeroPos(void);
private:
	/**
 * @brief Motor Externed
 */
	MsgQueue *queue;
    int wheel_c;
	int motor_c;
	int move_c;
	int level;
	MotorTypedef Motor_L = { .NodeID = 5 };  /* 左行走 */
	MotorTypedef Motor_R = { .NodeID = 6 }; /* 右行走 */
	MotorTypedef Motor_LIFT = { .NodeID = 7 };  /* 左行走 */
	MotorTypedef Motor_TURN = { .NodeID = 8 };
	u8 MotorError_Flag;
	pthread_t recieve_wheel_thread;
	pthread_t send_wheel_thread;
	pthread_mutex_t recieve_left_wheel;
	pthread_mutex_t recieve_right_wheel;
	static void* recieve_wheel(void* param);
	u8 *wheel_data;
	can_frame fr_recieve;
	bool send_thread_running =true;
	bool recieve_thread_running =true;
	int s;
	double velocity;
	double angular;
	float lift_height;
	bool is_arrive;
	int dir = -1;//0为下降，1为上升，-1为待命
};

#endif /* MOTOR_H_ */
