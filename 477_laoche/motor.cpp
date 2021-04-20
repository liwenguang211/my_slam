/* 步科驱动器CanOpen协议 */

#include "motor.h"
#include <iostream>
#include <math.h>
#include "agv.h"

/**
 * @brief 电机使能
 *
 * @return
 */
LiftMachineTypedef LiftMachine_zeroPosture = { 0, 0, 0, 0 };
LiftMachineTypedef LiftMachine_currentPosture = { 0, 0, 0, 0 };
LiftMachineTypedef LiftMachine_expectPosture = { 0, 0, 0, 0 };
const int cmd[2] = {0xc103,0xc104};//0xc103,0xc104分别为通道顶升电机上升或下降到达的信号值
Motor::Motor(MsgQueue *queue, int motor_chan, int wheel_chan, int move_chan)
{
	u8 i,ret;
	s = init_can();
	for (i = 0; i < 4; i++)
	{
		SwitchPDO(Motor_R);
		SwitchPDO(Motor_L);
		SwitchPDO(Motor_LIFT);
		SwitchPDO(Motor_TURN);

		motorSetSpeedMode(Motor_R);
		motorSetSpeedMode(Motor_L);
		motorSetPositionMode(Motor_LIFT);
		motorSetPositionMode(Motor_TURN);

		
	}
	//ret = LiftMachine_gotoZeroPos();
	motor_c = motor_chan;
	wheel_c = wheel_chan;
	move_c = move_chan;
	velocity = 0;
	angular = 0;
	lift_height = 40;
	this->queue = queue;
	queue->add_listener(motor_c, this);
	queue->add_listener(move_c, this);
	//pthread_create(&send_wheel_thread, NULL, send_wheel, this);
	pthread_create(&recieve_wheel_thread, NULL, recieve_wheel, this);
	recieve_thread_running = true;
}

/**
 * @brief 电机数据解析
 * @param thisMotor	电机结构体指针
 *
 * @return
 */
Motor::~Motor()
{
	//pthread_join(send_wheel_thread, NULL);
	pthread_join(recieve_wheel_thread, NULL);
	queue->remove_listener(motor_c, this);
	queue->remove_listener(move_c, this);
	end_can(s);
}

void Motor::MotorDataAnalyze(MotorTypedef *thisMotor, u8 *data)
{

	thisMotor->StatusWord = (data[0] | (data[1] << 8));
	thisMotor->ActualPosition = (data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24));

	thisMotor->EnableStatus = (u8)(thisMotor->StatusWord & 0x04) >> 2;
	thisMotor->Fault = (u8)(thisMotor->StatusWord & 0x08) >> 3;

	//电机故障时，将MotorError_Flag置1
	if (thisMotor->Fault == 1)
		MotorError_Flag = 1;

	thisMotor->timeStamp = get_current_time_us();
	//printf("%x ,%x ,%x ,%x ,%ld\n", thisMotor->StatusWord, thisMotor->ActualPosition, thisMotor->EnableStatus, thisMotor->Fault, thisMotor->timeStamp);
}

/**
 * @brief 开启站号
 *
 * @param	nodeID 站号
 * @return
 */
void Motor::SwitchPDO(MotorTypedef thisMotor)
{

	u8 data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	data[1] = thisMotor.NodeID;
	int ret;
	//CanSend(0x000, data);
	ret = can_send(s, 0x000, data);
	//if(ret < 0)
	//printf("canid not ready!\n");
}

/**
 * @brief 设为速度模式
 *
 * @param	nodeID 站号
 * @return
 */
void Motor::motorSetSpeedMode(MotorTypedef thisMotor)
{
	int ret;
	u8 data[8] = {0x0F, 0X00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};

	ret = can_send(s, (0x200 + thisMotor.NodeID), data);
	//if(ret < 0)
	//printf("speed_model not ready!\n");
	//CanSend(s,(0x200 + thisMotor.NodeID), data);
}

/**
 * @brief 设为位置模式
 *
 * @param	nodeID 站号
 * @return
 */
void Motor::motorSetPositionMode(MotorTypedef thisMotor)
{
	int ret;
	u8 data[8] = {0x3F, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
	ret = can_send(s, (0x200 + thisMotor.NodeID), data);
	//if(ret < 0)
	//printf("position_model not ready!\n");
	//CanSend((0x200 + thisMotor.NodeID), data);
}

/**
 * @brief 下发速度
 *
 * @param	nodeID 站号
 * @param	speed 速度
 * @return
 */
void Motor::motorSpeedControl(MotorTypedef thisMotor)
{

	int speed, ret;

	u8 data[8] = {0x00, 0X00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	speed = thisMotor.SpeedRatio;

	data[0] = (u8)speed;
	data[1] = (u8)(speed >> 8);
	data[2] = (u8)(speed >> 16);
	data[3] = (u8)(speed >> 24);

	// for (int i = 0; i < 8; i++)
	// 	printf("%x\t", data[i]);
	// printf("\n");

	ret = can_send(s, (0x300 + thisMotor.NodeID), data);
	//if(ret < 0)
	//printf("speed_control_model not ready!\n");
	//CanSend((0x300 + thisMotor.NodeID), data);
}

/**
 * @brief 下发目标位置
 *
 * @param	nodeID 站号
 * @param	speed 速度
 * @param	position 目标位置
 * @return
 */
void Motor::motorPositionControl(MotorTypedef thisMotor)
{

	u32 speed;
	int position, ret;

	u8 data[8] = {0x00, 0X00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	speed = abs(thisMotor.SpeedRatio);

	data[0] = (u8)(speed);
	data[1] = (u8)(speed >> 8);
	data[2] = (u8)(speed >> 16);
	data[3] = (u8)(speed >> 24);

	position = thisMotor.ExpectPositionRatio;
	data[4] = (u8)(position);
	data[5] = (u8)(position >> 8);
	data[6] = (u8)(position >> 16);
	data[7] = (u8)(position >> 24);

	ret = can_send(s, (0x400 + thisMotor.NodeID), data);
	//if(ret < 0)
	//printf("position_control_model not ready!\n");
	//CanSend((0x400 + thisMotor.NodeID), data);
}

/**
 * @brief 电机驱动
 *
 * @return
 */
// void Motor::MotorRun(void)
// {

// 	motorSpeedControl(Motor_L);
// 	motorSpeedControl(Motor_R);
// }

/**
 * @brief 电机停止
 *
 * @return
 */
void Motor::MotorStop(void)
{

	u8 i;

	for (i = 0; i < 4; i++)
		MotorRun(0, 0);
}

void Motor::recieve_message(const int channel, char *buf, const int size)
{

	if (channel == motor_c)
	{
		Speed_data data;
		data.from_char_array(buf, size);
		//printf("recv line_vel=%f,ang_vel=%f\n",data.vel_line,data.vel_ang);
		handle_obstacle_avoidance(data);
	}
	else if(channel == move_c)
	{
		MoveInstruction data;
		data.from_char_array(buf, size);
		//printf("recv line_vel=%f,ang_vel=%f\n",data.vel_line,data.vel_ang);
		
		// if(!data.lift.height)
		// 	printf("请设置顶升高度!当前顶升高度上限为%d\n",data.lift.height);
		// else
			
		if (data.lift.dir == 1) //顶升上升
		{
			dir = 1;
			Lift_MotorRise();
		}
		else if (data.lift.dir == 0)
		{
			dir = 0;
			Lift_MotorFall();
		}	
	}
	// printf("odom insert imu data %ld\n", data.timestamp_us);
}
void *Motor::recieve_wheel(void *param)
{
	//printf("ok1\n");
	Motor *re_motor = (Motor *)param;
	int ret;
	//printf("ok2\n");
	//printf("%d\n", re_motor->s);
	int j = 0, k = 0;

	//printf("%x\n",re_motor->fr_recieve.can_id);
	while (re_motor->recieve_thread_running)
	{
		ret = can_receive(re_motor->s, &(re_motor->fr_recieve));
		//printf("aaaaaaaaaaaaaaaaaaaaaaaaaaa%x\n",re_motor->fr_recieve.can_id);
		switch (re_motor->fr_recieve.can_id)
		{
		case 0x185:
		{
			//printf("ok30000000000000000000000000000000000000000\n");
			re_motor->MotorDataAnalyze(&re_motor->Motor_L, re_motor->fr_recieve.data);
			std::vector<char> tmp;
			WheelSensorData data;
			data.timestamp = re_motor->Motor_L.timeStamp;
			data.data = -re_motor->Motor_L.ActualPosition;
			data.type = WHEEL_DATA_TYPE_LEFT;
			//std::cout<<data.data<<std::endl;
			data.to_char_array(&tmp);
			re_motor->queue->send(re_motor->wheel_c, tmp.data(), tmp.size());
		}
		break;
		case 0x186:
		{
			//printf("ok4999999999999999999999999999999999999\n");

			re_motor->MotorDataAnalyze(&re_motor->Motor_R, re_motor->fr_recieve.data);
			std::vector<char> tmp;
			WheelSensorData data;
			data.timestamp = re_motor->Motor_R.timeStamp;
			data.data = re_motor->Motor_R.ActualPosition;
			data.type = WHEEL_DATA_TYPE_RIGHT;
			//std::cout<<data.data<<std::endl;
			data.to_char_array(&tmp);
			re_motor->queue->send(re_motor->wheel_c, tmp.data(), tmp.size());
		}
		break;
		case 0x187:
		{
			re_motor->MotorDataAnalyze(&re_motor->Motor_LIFT, re_motor->fr_recieve.data);
			re_motor->is_arrive = re_motor->LiftMachine_TaskComplete();
			//printf("111111111111111111%d\n",re_motor->is_arrive);
			if (re_motor->is_arrive == true)
			{
				if (re_motor->queue != nullptr && re_motor->dir == 1)
				{
					re_motor->queue->send(CHANNEL_ARRIVE, (char *)(cmd), sizeof(int));
					re_motor->dir = -1;
					std::cout<<"顶升完成!!!\n"<<std::endl;
				}
				else if (re_motor->queue != nullptr && re_motor->dir == 0)
				{
					re_motor->queue->send(CHANNEL_ARRIVE, (char *)(cmd+1), sizeof(int));
					re_motor->dir = -1;
					std::cout<<"顶升下降完成!!!\n"<<std::endl;
				}
			}
		}
			break;
		case 0x188:
		{
			re_motor->MotorDataAnalyze(&re_motor->Motor_TURN, re_motor->fr_recieve.data);
			re_motor->is_arrive = re_motor->LiftMachine_TaskComplete();
			//printf("22222222222222%d\n",re_motor->is_arrive);
			if (re_motor->is_arrive == true)
			{
				if (re_motor->queue != nullptr && re_motor->dir == 1)
				{
					re_motor->queue->send(CHANNEL_ARRIVE, (char *)(cmd), sizeof(int));
					re_motor->dir = -1;
					std::cout<<"顶升完成!!!\n"<<std::endl;
				}
				else if (re_motor->queue != nullptr && re_motor->dir == 0)
				{
					re_motor->queue->send(CHANNEL_ARRIVE, (char *)(cmd+1), sizeof(int));
					re_motor->dir = -1;
					std::cout<<"顶升下降完成!!!\n"<<std::endl;

				}
			}
		}
		break;
		default:
			printf("wheel data error!!!\n");
			break;
		}
	}
}

// m/s   rad/s
void Motor::MotorRun(Speed_data data)
{

	velocity = data.vel_line;
	angular = data.vel_ang;

	float vl, vr, l, r, ratio, reduce;

	l = 0.25; /* 輪子到中心距離m   这里要加入文件读取函数 */
	reduce = 9;
	ratio = 2731;
	r = 0.072;

	vl = velocity - angular * l;
	vr = velocity + angular * l;

	vr *= 60 / (2 * M_PI * r) * ratio * reduce;
	vl *= -60 / (2 * M_PI * r) * ratio * reduce;

	Motor_L.SpeedRatio = vl;
	Motor_R.SpeedRatio = vr;

	motorSpeedControl(Motor_L);
	motorSpeedControl(Motor_R);
}

void Motor::MotorRun(double v, double omega)
{
	float vl, vr, l, r, ratio, reduce;

	l = 0.25; /* 輪子到中心距離m   这里要加入文件读取函数 */
	reduce = 9;
	ratio = 2731;
	r = 0.072;

	vl = v - omega * l;
	vr = v + omega * l;

	vr *= 60 / (2 * M_PI * r) * ratio * reduce;
	vl *= -60 / (2 * M_PI * r) * ratio * reduce;

	Motor_L.SpeedRatio = vl;
	Motor_R.SpeedRatio = vr;

	motorSpeedControl(Motor_L);
	motorSpeedControl(Motor_R);
}
/*多级避障处理*/
void Motor::handle_obstacle_avoidance(Speed_data data)
{
	//printf("ok222222222222222222\n");
	level = get_global_agv_instance()->collision_level;
	//printf("ok111\n");
	if (level == 1)
		MotorRun(0, 0);
	else if (level == 2)
	{
		velocity = data.vel_line;
		angular = data.vel_ang;
		velocity = velocity / 2;
		angular = angular / 2;
		MotorRun(velocity, angular);
	}
	else if (level == 3)
	{
		MotorRun(data);
		printf("Action!!!\n");
	}
	else
		MotorRun(data);
}

/*顶升电机抬升动作*/
void Motor::Lift_MotorRise(void)//加参数
{
	Lift_Motor_setExpectpose(25, LiftMachine_expectPosture.Angle);
	Motor_LIFT.SpeedRatio = 800 * 17896; /* r/min */
	Motor_TURN.SpeedRatio = 800 * 17896;
	motorPositionControl(Motor_LIFT);
	motorPositionControl(Motor_TURN);

}

/*顶升电机下降动作*/
void Motor::Lift_MotorFall(void)//加参数
{
	Lift_Motor_setExpectpose(0, LiftMachine_expectPosture.Angle);
	Motor_LIFT.SpeedRatio = 800 * 17896; /* r/min */
	Motor_TURN.SpeedRatio = 800 * 17896;
	motorPositionControl(Motor_LIFT);
	motorPositionControl(Motor_TURN);

}

/*旋转电机动作*/
void Motor::Turn_MotorRun(float expect_angle, float omega)//加参数
{
	float speed_ratio = 0;

	Lift_Motor_setExpectpose(LiftMachine_expectPosture.Height, expect_angle);

	speed_ratio = omega / (2 * M_PI) * 60 * 2.62 * 20 * 17896;
	Motor_LIFT.SpeedRatio = speed_ratio;
	Motor_TURN.SpeedRatio = speed_ratio;
	motorPositionControl(Motor_LIFT);
	motorPositionControl(Motor_TURN);

}

/*旋转、顶升电机停止*/
void Motor::Turn_MotorStop(void)//加参数
{
	Motor_LIFT.SpeedRatio = 0;
	Motor_TURN.SpeedRatio = 0;
	motorPositionControl(Motor_LIFT);
	motorPositionControl(Motor_TURN);

}
/*获取旋转、顶升电机绝对零点位置编码器值*/
void Motor::LT_Motor_getZeroPose(void)
{
	LiftMachine_zeroPosture.ActualPosition_L = 0;/////////////////////////这里还没改
	LiftMachine_zeroPosture.ActualPosition_T = 0;
	LiftMachine_zeroPosture.Height = 0;
	LiftMachine_zeroPosture.Angle = 0;
	
}
/*计算旋转、顶升电机实时姿态*/
void Motor::LT_Motor_calcCurrentPose(void)
{
	float n;
	float L_AP, T_AP, H, A;

	LiftMachine_currentPosture.ActualPosition_L = Motor_LIFT.ActualPosition;
	LiftMachine_currentPosture.ActualPosition_T = Motor_TURN.ActualPosition;
	
	//printf("555555555555555%d,%d\n",Motor_TURN.ActualPosition,Motor_LIFT.ActualPosition);
	/* 相对零点 编码器值 */
	L_AP = LiftMachine_currentPosture.ActualPosition_L - LiftMachine_zeroPosture.ActualPosition_L;
	T_AP = LiftMachine_currentPosture.ActualPosition_T - LiftMachine_zeroPosture.ActualPosition_T;

	/* 计算 转换系数  * 编码器分辨率 * 一级减速比 * 二级减速比   (转换后单位 r , 10mm螺距 , 1转360°) */
	n = 65536 * 20 * 2.62;
	H = (L_AP / n + T_AP / n) * (-10.0);
	A = (T_AP / n) * 360.0;

	/* 当前实时姿态 mm  ° */
	LiftMachine_currentPosture.Height = H;
	LiftMachine_currentPosture.Angle = A;
}

/*设置旋转期望姿态对应编码器值（相对于此时刻的这个相对零点）*/
void Motor::Lift_Motor_setExpectpose(float expect_H, float expect_A)
{
	float n;
	float L_AP, T_AP, H, A;
	u16 angle_range = 360;

	/* 限制小车顶升高度 极限位置 40mm */
	if (expect_H < 0)
		expect_H = 0;
	else if(expect_H > lift_height)
		expect_H = lift_height;

	/* 限制转角范围  */
	while (expect_A > angle_range)
		expect_A -= 360;
	while (expect_A < -angle_range)
		expect_A += 360;

	LiftMachine_expectPosture.Height = expect_H;
	LiftMachine_expectPosture.Angle = expect_A;
	H = expect_H;
	A = expect_A;

	/* 计算 转换系数   编码器分辨率 * 一级减速比 * 二级减速比   (转换后单位 r , 10mm导程 , 1转360°) */
	n = 65536 * 20 * 2.62;
	L_AP = (H / (-10.0) - (A / 360.0)) * n;
	T_AP = (A / 360.0) * n;

	/* 加上 零点位置值 */
	L_AP += LiftMachine_zeroPosture.ActualPosition_L;
	T_AP += LiftMachine_zeroPosture.ActualPosition_T;

	/* 根据期望姿态 计算 期望下发数字量 */
	LiftMachine_expectPosture.ActualPosition_L = L_AP;
	LiftMachine_expectPosture.ActualPosition_T = T_AP;

	/* 驱动器下发目标位置值 */
	{
		Motor_LIFT.ExpectPositionRatio = L_AP;
		Motor_TURN.ExpectPositionRatio = T_AP;
	}
	
}

/*顶升机构任务完成状态
* 任务完成返回1；未完成返回0
*/
bool Motor::LiftMachine_TaskComplete(void) {
	bool result = false;
	LT_Motor_calcCurrentPose();
	if (fabs(LiftMachine_currentPosture.Angle - LiftMachine_expectPosture.Angle) < 1
			&& fabs(LiftMachine_currentPosture.Height - LiftMachine_expectPosture.Height) < 2)
		result = true;
	//printf("111111111111111111111111111111%f,%f\n",LiftMachine_currentPosture.Angle - LiftMachine_expectPosture.Angle,LiftMachine_currentPosture.Height - LiftMachine_expectPosture.Height);

	return result;
}

/*初始化顶升、旋转电机归置零位*/
u8 Motor::LiftMachine_gotoZeroPos(void) {

	Lift_Motor_setExpectpose(0, 0);

	Motor_LIFT.SpeedRatio = 100 * 17896;      //r/min 500
	Motor_TURN.SpeedRatio = 100 * 17896;
	motorPositionControl(Motor_LIFT);
	motorPositionControl(Motor_TURN);

	return 0;
}
