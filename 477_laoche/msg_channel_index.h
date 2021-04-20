#ifndef MESSAGE_CHANNEL_INDEX_H
#define MESSAGE_CHANNEL_INDEX_H

#define CHANNEL_IGNORE  0   //不处理
#define CHANNEL_IMU     1   //IMU数据
#define CHANNEL_RADAR   2   //激光雷达数据
#define CHANNEL_WHEEL   3   //轮子编码器数据
#define CHANNEL_MOVE    4   //移动指令
#define CHANNEL_POSE    5   //实时位姿数据
#define CHANNEL_ODOM    6   //实时位姿变化量
#define CHANNEL_CONTROL 7   //外部控制指令	//暂未使用
#define CHANNEL_ACTION  8   //额外动作指令	//暂未使用
#define CHANNEL_FINISH  9   //任务完成信息	//暂未使用
#define CHANNEL_TIMER   10  //定时器		//暂未使用
#define CHANNEL_ARRIVE  11  //int,1:Turn,2:Arrive
#define CHANNEL_STATE  	12	
#define CHANNEL_STOP  	13
#define CHANNEL_MOTOR_CTL    14   //伺服控制指令
#define CHANNEL_ERROR   233 //错误信息输出

#endif
