#include "curve_run_fn.h"
#include "agv.h"
curve_run::curve_run(MsgQueue *queue,int move_chan)     
{
	this->queue = queue;
	chan_move = move_chan;
	queue->add_listener(move_chan, this);
	queue->add_listener(CHANNEL_ARRIVE, this);
	line_cnt = 0;
	rotate_cnt = 0;
  	
    task_status = ROBOT_TASK_STS_NONE;
	pthread_create(&manu_run_thread, nullptr, manual_run_thread_, this);
}
curve_run::curve_run()      
{
	line_cnt = 0;
	rotate_cnt = 0;
	pthread_create(&manu_run_thread, nullptr, manual_run_thread_, this);
}
curve_run::~curve_run()
{
	pthread_join(manu_run_thread, NULL);
	queue->remove_listener(chan_move, this);
}
void * curve_run::manual_run_thread_(void * param)
{
  printf("manual run thread starting!!!!!!!!!!!!!!!!!!!!!!\n");
	curve_run *ptr = (curve_run *)param;
	bool mode_changed = false;
	int last_mode = ptr->mode_manual_auto;
	int count_time=0;
	while(true)
	{
		if(ptr->mode_manual_auto !=last_mode) 
		mode_changed = true;
		
		///////////////////如果发生模式切换则清空当前所有的速度指令，避免当前模式的速度代入到新模式中
		if(mode_changed == true)
		{
		mode_changed = false;
		if(last_mode==MODE_AUTO)
		ptr->mode_auto2manual();
		last_mode = ptr->mode_manual_auto;
		}
		count_time++;
		
		if(!(count_time % 20))
		{
		/////////////////////////////////////////////////刷新机器人置信度
    ;
		}
		if(ptr->mode_manual_auto == MODE_AUTO) {usleep(20000);continue;}//////如果是自动运行模式，直接返回
		ptr->manual_run();
		//printf("手动\n");
		usleep(20000);
	}

}
void curve_run::manual_run()
{
	//printf("手动11111111111111111111\n");
	//if(mode_manual_auto == MODE_MANUAL||(get_global_agv_instance()->get_system_state() == SYSTEM_STATE_MAPPING))
	
	double max_line_speed = nav_vel_line*vel_ratio;
	double max_angle_speed = nav_vel_angle*vel_ratio;

	if(mode_manual_auto == MODE_MANUAL||(get_global_agv_instance()->get_system_state() == SYSTEM_STATE_MAPPING))
	if(vel_ratio>0.5)
	{max_line_speed = nav_vel_line*0.45;max_angle_speed = nav_vel_angle*0.45;}
	double line_acc = nav_acc_line;
	double line_dec = nav_dec_line;
	double angle_acc = nav_acc_angle;
	double angle_dec = nav_dec_angle;
	double line_vel,angle_vel;
	float delta_t = 0.03;
  if(line_cnt>0)line_cnt--;
  else
  {dir_f=false;dir_b=false;}
  if(rotate_cnt>0)rotate_cnt--;
  else
  {dir_l=false;dir_r=false;}
	if(dir_f)
  {
	if(manual_run_lvel<max_line_speed)
	manual_run_lvel +=line_acc*delta_t;
	else manual_run_lvel = max_line_speed;
	////printf("手动11111111111111111111%f\n",manual_run_lvel);
	}
	if(dir_b)
  {
	if(manual_run_lvel>-max_line_speed)
	manual_run_lvel -=line_acc*delta_t;
	else manual_run_lvel = -max_line_speed;
	}
	if(dir_l)
  {
	if(manual_run_rvel<max_angle_speed)
	manual_run_rvel +=angle_acc*delta_t;
	else manual_run_rvel = max_angle_speed;
	}
	if(dir_r)
  {
	if(manual_run_rvel>-max_angle_speed)
	manual_run_rvel -=angle_acc*delta_t;
	else manual_run_rvel = -max_angle_speed;
	}


	if(!dir_f&&(!dir_b))
	{
	if(manual_run_lvel>line_dec*delta_t) manual_run_lvel-=line_dec*delta_t*1.5;
	else if(manual_run_lvel<-line_dec*delta_t) manual_run_lvel+=line_dec*delta_t*1.5;
	else manual_run_lvel = 0.0;
	}
	if(!dir_l&&(!dir_r))
	{
	//if(manual_run_rvel>angle_dec*delta_t) manual_run_rvel-=angle_dec*delta_t;
	//else if(manual_run_rvel<-angle_dec*delta_t) manual_run_rvel+=angle_dec*delta_t;
	//else manual_run_rvel = 0.0;
	manual_run_rvel = 0.0;
	}
  line_vel = manual_run_lvel;
  angle_vel = manual_run_rvel;
  Speed_data speed_send(0,0);
	//printf("输出线速度=%f,角速度=%f\n",manual_run_lvel,manual_run_rvel);
  speed_send.vel_line = manual_run_lvel;
  speed_send.vel_ang = manual_run_rvel;
  std::vector<char> tmp;
  speed_send.to_char_array(&tmp);
  queue->send(CHANNEL_MOTOR_CTL, tmp.data(), tmp.size());
}
bool curve_run::mode_auto2manual()/////从自动模式切换至手动模式的处理
{
  manual_run_lvel = auto_run_lvel;
  manual_run_rvel = auto_run_rvel;
  auto_run_lvel = 0;
  auto_run_rvel = 0;
  line_cnt = 0;
  rotate_cnt = 0;
	return true;
	if(task_status == ROBOT_TASK_STS_RUNNING)
	task_status = ROBOT_TASK_STS_SUSPENDING;
}
void curve_run::recieve_message(const int channel, char *buf, const int size) {
    if(channel == chan_move) {
        MoveInstruction ins;
    	ins.from_char_array(buf, size);
	if (ins.type == MOVE_INSTRUCTION_TYPE_SPEED)
	{
		if(fabs(ins.speed.speed)>0)
		{if(line_cnt<10)line_cnt=10;
		if(ins.speed.speed>0)
		dir_f=true;
		else
		dir_b=true;
		mode_manual_auto = MODE_MANUAL;
		//printf("接收5555速度指令l=%f,r=%f\n",manual_run_lvel,manual_run_rvel);
		}
		if(fabs(ins.speed.turn_left)>0)
		{if(rotate_cnt<10)rotate_cnt=10;
		if(ins.speed.turn_left>0)
		dir_l=true;
		else
		dir_r=true;
		mode_manual_auto = MODE_MANUAL;
		//printf("接收7777速度指令l%f,r=%f\n",manual_run_lvel,manual_run_rvel);
		}
	}
        // printf("odom insert imu data %ld\n", data.timestamp_us);
    }
	else if(channel == CHANNEL_ARRIVE)
	{

		printf("轨迹驱动接口程序---------接收到达指令\n");
		int cmd=*((int*)buf);
		switch(cmd){
			
			case 0xc103://上报顶升上升完成
				printf("轨迹驱动接口程序---------上报顶升上升完成\n");
				lift_up_finished=true;
				break;
			case 0xc104://上报顶升下降完成
				printf("轨迹驱动接口程序---------上报顶升下降完成\n");
				lift_down_finished=true;
				break;
			default:
				break;
		}
	}

}
void curve_run::send_lift_msg(int dir){
	MoveInstruction move;
	move.type = MOVE_INSTRUCTION_TYPE_LIFT;
	move.lift.dir=dir;
	std::vector<char> data;
	move.to_char_array(&data);
	get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
}
T_TRAJ_ curve_run::ptp2traj(Position ps,Position pe,float speed,char inverse_s,int traj_typ)/////点到点组合成轨迹参数
{
	T_TRAJ_ ptraj;
  if(fabs(speed)<0.001||fabs(speed)>1.0) speed = 0.3;
	ptraj.PTP_OR_CURVE = TRAJ_ISPTP;
	ptraj.traj_type = traj_typ;
	ptraj.tarjp.CURVE_INITIAL_POSE = ps;
	ptraj.tarjp.CURVE_TARGET_POSE = pe;
  
	//ptraj.tarjp.length_read = dis
	dis_between(ps,pe,ptraj.tarjp.length);
	ptraj.tarjp.length;
  if(speed>sqrt((ptraj.tarjp.length-nav_slow_dis)*nav_dec_line))
	speed = sqrt((ptraj.tarjp.length-nav_slow_dis)*nav_dec_line);
  ptraj.isinverse = inverse_s;
	ptraj.vel_line = speed;
	ptraj.tarjp.stat_index = NONSENSEVAL;
	ptraj.tarjp.end_index = NONSENSEVAL;
	return ptraj;
}
T_TRAJ_ curve_run::ptp2curvetraj(Position pst,Position ped,float speed,char inverse_t,int traj_typ)/////点到点组合成轨迹参数
{
	T_TRAJ_ cur_trajp;
  if(fabs(speed)<0.001||fabs(speed)>1.0) speed = 0.3;
  cur_trajp.tarjp.CURVE_TARGET_POSE = ped;
  cur_trajp.tarjp.CURVE_INITIAL_POSE = pst;

  cur_trajp.tarjp.CURVE_CONTROLP1 = pst;
  cur_trajp.tarjp.CURVE_CONTROLP1.x += (ped.x-pst.x)/3;
  cur_trajp.tarjp.CURVE_CONTROLP1.y += (ped.y-pst.y)/3;
  cur_trajp.tarjp.CURVE_CONTROLP2 = pst;
  cur_trajp.tarjp.CURVE_CONTROLP2.x += (ped.x-pst.x)*2/3;
  cur_trajp.tarjp.CURVE_CONTROLP2.y += (ped.y-pst.y)*2/3;
  getBezier_length(cur_trajp.tarjp);/////求解该曲线的长度，等参数
  cur_trajp.isinverse = inverse_t;

  printf("33333333cur traj dirction inverse=%d,%d\n",cur_trajp.isinverse,inverse_t);
  if(speed>sqrt((cur_trajp.tarjp.length-nav_slow_dis)*nav_dec_line))
	speed = sqrt((cur_trajp.tarjp.length-nav_slow_dis)*nav_dec_line);
	
  cur_trajp.vel_line = speed;
  cur_trajp.PTP_OR_CURVE = TRAJ_ISCURVE;
  

	return cur_trajp;
}
int curve_run::initial_path(vector <Position> pathpoints,vector <float> vel_points,vector <char> inverse_t,char do_act)
{
	
  //for (int ii_tmp=0;ii_tmp<inverse_t.size();ii_tmp++)
  //	printf("22222222222222cur traj direction inverse=%d,vel=%f\n",inverse_t[ii_tmp],vel_points[ii_tmp]);
  
  if (task_status != ROBOT_TASK_STS_NONE) ///ir
  {////if the task is already running,firstly stop the thread, then start one another thread;

    mode_auto2manual();
    mode_manual_auto = MODE_MANUAL;
    usleep(20000);
    pthread_join(curve_run_thread, NULL);
    while(manual_run_lvel>FUZZY_MINI||manual_run_rvel>FUZZY_MINI)/////等待上一步的任务完全停止下来
		{
		usleep(20000);
		}
		usleep(500000);
  }
  LOOKAHEADFNISED = false;
  TRAJDONE = false;
  cancel_path();
  if(set_path(pathpoints,vel_points,inverse_t,do_act)!=TRAJ_LOOKAHEAD_OK) return -1;
  traj_par_init();
  task_status = ROBOT_TASK_STS_SUSPENDING;
  mode_manual_auto = MODE_AUTO;
  printf("traj look ahead successful,then start the curve run thread\n");
	pthread_create(&curve_run_thread, nullptr, curve_move_thread_, this);
  return 1;
}
bool curve_run::cancel_path()//////轨迹取消
{
  vector <T_TRAJ_> myspace_traj_temp;////从下发指令推算出的轨迹
  if(mode_manual_auto != MODE_MANUAL) /////接收指令后，切换至手动运行模式 
	{mode_manual_auto = MODE_MANUAL;mode_auto2manual();usleep(10000);}	
		
  myspace_traj_temp.swap(myspace_traj);
	myspace_traj.clear();  
  TRAJDONE = false;
  task_status = ROBOT_TASK_STS_NONE;
  LOOKAHEADFNISED = false;
	return true;
}
bool curve_run::stop_path()/////暂停轨迹
{
    vel_start = 0;
		running_time = 0;
		if(mode_manual_auto != MODE_MANUAL) /////接收指令后，切换至手动运行模式 
		{mode_manual_auto = MODE_MANUAL;mode_auto2manual();usleep(10000);}
		usleep(10000);
		task_status = ROBOT_TASK_STS_SUSPENDING;
	return true;
}
bool curve_run::continue_path()/////继续轨迹
{
    auto_run_lvel = 0.0;
		auto_run_rvel = 0.0;
		vel_start = auto_run_lvel;
		cmd_line_vel = vel_start;
		running_time = 0;//////运动计时清零，暂停后再启动，从0开始加速
		mode_manual_auto = MODE_AUTO;////接收路径规划指令，自动切换到自动模式
		usleep(20000);
    task_status = ROBOT_TASK_STS_SUSPENDING;
	return true;
}



int curve_run::set_path(vector <Position> pathpoints,vector <float> vel_points,vector <char> inverse_s,char do_act)
{
  int idex= pathpoints.size();
printf(" pahtpoints received number=%d,\n",idex);
  int total_pnum = idex;
  //for (int ii_tmp=0;ii_tmp<inverse_s.size();ii_tmp++)
//	printf("11111111111111cur traj direction inverse=%d,vel=%f\n",inverse_s[ii_tmp],vel_points[ii_tmp]);
  if(total_pnum<=0)return TRAJ_LKAH_PATHPOINT_ERROR;;
  if(vel_points.size()!=total_pnum||inverse_s.size()!=total_pnum&&total_pnum) return TRAJ_LKAH_PATHPOINT_ERROR;//////下发的路径点指令出错
  realpose_map = get_global_agv_instance()->expolate_current_position();
  target_pos = pathpoints[total_pnum-1];
  Position  zero_point;
  Position p1;
	Position p2;
	Position p3;
  zero_point.x = 0;
  zero_point.y = 0;
  zero_point.theta =0;
  printf("total path points=%d target pointsx=%f  y=%f  th=%f\n",total_pnum,target_pos.x,target_pos.y,target_pos.theta);
  int first_idex = 0;
	int second_idex = 1;
  bool bp1=true;
	bool bp2=true;
	bool bp3=true;
	double theta;/////两条轨迹之间的夹角
  T_TRAJ_ cur_trajp;/////////
	T_TRAJ_ cur_trajp1;/////////
	memset(&cur_trajp, 0, sizeof(T_TRAJ_));	
	memset(&cur_trajp1, 0, sizeof(T_TRAJ_));
  if(dis_among(pathpoints[first_idex],zero_point,0.001)) 
  {
    bp1 = false;
    return TRAJ_LKAH_PATHPOINT_ERROR;//////下发的路径点指令出错
  }
  while(dis_among(pathpoints[first_idex],pathpoints[second_idex],0.01))////找出距离第一个点大于0.01m的点作为第二个路径点
	{
    second_idex++;
    if(second_idex<idex) 
    bp2=true;
    else
    {bp2=false;break;}
  }
 
  Position pnear;
  if(bp1==true&&bp2==true)
  cur_trajp = ptp2curvetraj(pathpoints[first_idex],pathpoints[second_idex],vel_points[second_idex],inverse_s[second_idex],TRAJ_COMPLETE);
	else if(bp1==true&&bp2==false)
	cur_trajp = ptp2traj(realpose_map,pathpoints[first_idex],vel_points[first_idex],inverse_s[first_idex],TRAJ_COMPLETE);

	if(bp1==true&&bp2==false)/////如果仅仅发送了一个点
	{//////仅仅发送了一个单点的任务
	cur_trajp1 = cur_trajp;
	cur_trajp1.traj_type = TRAJ_COMPLETE;//////
	cur_trajp1.trisend = TRAJ_ISENDDING;
	myspace_traj.push_back(cur_trajp1);
	cur_trajp1.trisend = TRAJ_FINISHED;//////
	myspace_traj.push_back(cur_trajp1);/////将最终的结束路径段压入到轨迹缓冲
	return TRAJ_LOOKAHEAD_OK;//////返回
	}
	else
	{
	//////////////////求解当前点到轨迹上最近距离的点
  pnear = curp2curve(realpose_map,cur_trajp.tarjp,cur_trajp.PTP_OR_CURVE);///计算当前点到距离曲线上最近点的ptp路径
	if(sqrt(sqsum(realpose_map,pnear))>0.5)
	{
    cur_trajp1 = ptp2traj(realpose_map,pnear,vel_points[first_idex],inverse_s[first_idex],TRAJ_COMPLETE);
	  myspace_traj.push_back(cur_trajp1);/////将该路径压入到轨迹缓冲
	}
	
	cur_trajp.traj_type = TRAJ_COMPLETE;//////
	myspace_traj.push_back(cur_trajp);/////将曲线段路径压入到轨迹缓冲
	printf("计算得到的trajp1的ini_u=%d,trajp=%d\n",cur_trajp1.tarjp.init_u,cur_trajp.tarjp.init_u);
	
	}

  ///////最终压入一条结束路径，避免仅有一条路径，丢失结束段的情况
	memset(&cur_trajp, 0, sizeof(T_TRAJ_));	

	for(idex = second_idex;idex<total_pnum-1;idex++)
	{
		memset(&cur_trajp, 0, sizeof(T_TRAJ_));		
		bp1 = true;bp2 = true; 
		if(idex < total_pnum)  bp1 = true; else bp1 = false;
		if((idex+1) < total_pnum)  bp2 = true; else bp2 = false;
		
		if(bp1 == false)/////第一个点就是空，则不压入当前轨迹
			break;
		if(bp1==true&&bp2==true)
		{//////
/////////////////////////////////////////////////////////////////
      cur_trajp = ptp2curvetraj(pathpoints[idex],pathpoints[idex+1],vel_points[idex+1],inverse_s[idex+1],TRAJ_COMPLETE);
			myspace_traj.push_back(cur_trajp);/////将曲线段路径压入到轨迹缓冲
		}
		else if(bp1==true&&bp2==false)
		{
			cur_trajp.trisend = TRAJ_FINISHED;//////
			myspace_traj.push_back(cur_trajp);/////将最终的结束路径段压入到轨迹缓冲	
		}
		//printf("第idex=%d段路径类型========%d,END=%d\n",idex,curve_para.linetype,cur_trajp.trisend);
	}
	///////最终压入一条结束路径，避免仅有一条路径，丢失结束段的情况
	memset(&cur_trajp, 0, sizeof(T_TRAJ_));	
	cur_trajp.trisend = TRAJ_FINISHED;//////
	myspace_traj.push_back(cur_trajp);/////将最终的结束路径段压入到轨迹缓冲
	printf("总路径段数+++++++++%d\n",myspace_traj.size());

/////////////////////////////////////////////运行到此处，轨迹已经全部压入轨迹缓冲，接下来需要计算各段轨迹的类型，起始，过渡，终止还是完整段。
	for(int jj=1;jj<myspace_traj.size();jj++)
	{
		///////////////////判断第一段的类型,针对仅仅有一个输入目标点的情况
		if(jj == 1)
		if(myspace_traj[1].trisend == TRAJ_FINISHED)
		{
			myspace_traj[0].traj_type = TRAJ_COMPLETE;
			myspace_traj[0].trisend = TRAJ_ISENDDING;
			printf("当前%d段是endding段\n",0);
		}

		/////////////////////判断前一轨迹段的类型和下一轨迹段是否是结束标识TRAJ_FINISHED
		if(myspace_traj[jj-1].traj_type == TRAJ_COMPLETE||myspace_traj[jj-1].traj_type == TRAJ_END)
		//////如果前一段是完整或者结束段
		{
			if(myspace_traj[jj+1].trisend == TRAJ_FINISHED)/////如果下一段是结束标识
			/////那么定位该段类型为完整段，且是标识为EDDING
			{
				myspace_traj[jj].traj_type = TRAJ_COMPLETE;
				myspace_traj[jj].trisend = TRAJ_ISENDDING;
				printf("当前%d段是endding段\n",jj);
			}
			else
			{/////如果不是结束标识，判断当前段与下一个段的夹角
				if(myspace_traj[jj].PTP_OR_CURVE == TRAJ_ISCURVE&&myspace_traj[jj+1].PTP_OR_CURVE == TRAJ_ISCURVE)
				{/////如果当前段是曲线，下一段是曲线
					p1 = myspace_traj[jj].tarjp.ED_LKAHP;
					p2 = myspace_traj[jj].tarjp.CURVE_TARGET_POSE;
					p3 = myspace_traj[jj+1].tarjp.ST_LKAHP;
					theta = angle_betw(p1,p2,p3);	
				}
				else if(myspace_traj[jj].PTP_OR_CURVE == TRAJ_ISCURVE&&myspace_traj[jj+1].PTP_OR_CURVE == TRAJ_ISPTP)
				{/////如果当前段是曲线，下一段是PTP
					p1 = myspace_traj[jj].tarjp.ED_LKAHP;
					p2 = myspace_traj[jj].tarjp.CURVE_TARGET_POSE;
					p3 = myspace_traj[jj+1].tarjp.CURVE_TARGET_POSE;
					theta = angle_betw(p1,p2,p3);	
				}
				else if(myspace_traj[jj].PTP_OR_CURVE == TRAJ_ISPTP&&myspace_traj[jj+1].PTP_OR_CURVE == TRAJ_ISCURVE)
				{/////如果当前段是PTP，下一段是曲线
					p1 = myspace_traj[jj].tarjp.CURVE_INITIAL_POSE;
					p2 = myspace_traj[jj].tarjp.CURVE_TARGET_POSE;
					p3 = myspace_traj[jj+1].tarjp.ST_LKAHP;
					theta = angle_betw(p1,p2,p3);	
				}
				else if(myspace_traj[jj].PTP_OR_CURVE == TRAJ_ISPTP&&myspace_traj[jj+1].PTP_OR_CURVE == TRAJ_ISPTP)
				{/////如果当前段是PTP，下一段是PTP
					p1 = myspace_traj[jj].tarjp.CURVE_INITIAL_POSE;
					p2 = myspace_traj[jj].tarjp.CURVE_TARGET_POSE;
					p3 = myspace_traj[jj+1].tarjp.CURVE_TARGET_POSE;
					theta = angle_betw(p1,p2,p3);	
				}
				////////如果两段轨迹间的夹角足够大，那么该段轨迹为起始段

				if(theta>0&&(myspace_traj[jj].isinverse==myspace_traj[jj+1].isinverse)) 
				myspace_traj[jj].traj_type = TRAJ_START;
				//////否则定义该段类型为完整段
				else myspace_traj[jj].traj_type = TRAJ_COMPLETE;
			}
		}
		else if(myspace_traj[jj-1].traj_type == TRAJ_START||myspace_traj[jj-1].traj_type == TRAJ_MIDDLE)
		{////如果前一段轨迹为起始或者过渡段
			if(myspace_traj[jj+1].trisend == TRAJ_FINISHED)/////如果下一段是结束标识
			/////那么定位该段类型为结束段，且是标识为EDDING
			{
				myspace_traj[jj].traj_type = TRAJ_END;
				myspace_traj[jj].trisend = TRAJ_ISENDDING;
				printf("当前段%d是endding2222段\n",jj);
			}
			else
			{/////如果不是结束标识，判断当前段与下一个段的夹角
				if(myspace_traj[jj].PTP_OR_CURVE == TRAJ_ISCURVE&&myspace_traj[jj+1].PTP_OR_CURVE == TRAJ_ISCURVE)
				{/////如果当前段是曲线，下一段是曲线
					p1 = myspace_traj[jj].tarjp.ED_LKAHP;
					p2 = myspace_traj[jj].tarjp.CURVE_TARGET_POSE;
					p3 = myspace_traj[jj+1].tarjp.ST_LKAHP;
					theta = angle_betw(p1,p2,p3);	
				}
				else if(myspace_traj[jj].PTP_OR_CURVE == TRAJ_ISCURVE&&myspace_traj[jj+1].PTP_OR_CURVE == TRAJ_ISPTP)
				{/////如果当前段是曲线，下一段是PTP
					p1 = myspace_traj[jj].tarjp.ED_LKAHP;
					p2 = myspace_traj[jj].tarjp.CURVE_TARGET_POSE;
					p3 = myspace_traj[jj+1].tarjp.CURVE_TARGET_POSE;
					theta = angle_betw(p1,p2,p3);	
				}
				else if(myspace_traj[jj].PTP_OR_CURVE == TRAJ_ISPTP&&myspace_traj[jj+1].PTP_OR_CURVE == TRAJ_ISCURVE)
				{/////如果当前段是PTP，下一段是曲线
					p1 = myspace_traj[jj].tarjp.CURVE_INITIAL_POSE;
					p2 = myspace_traj[jj].tarjp.CURVE_TARGET_POSE;
					p3 = myspace_traj[jj+1].tarjp.ST_LKAHP;
					theta = angle_betw(p1,p2,p3);	
				}
				else if(myspace_traj[jj].PTP_OR_CURVE == TRAJ_ISPTP&&myspace_traj[jj+1].PTP_OR_CURVE == TRAJ_ISPTP)
				{/////如果当前段是PTP，下一段是PTP
					p1 = myspace_traj[jj].tarjp.CURVE_INITIAL_POSE;
					p2 = myspace_traj[jj].tarjp.CURVE_TARGET_POSE;
					p3 = myspace_traj[jj+1].tarjp.CURVE_TARGET_POSE;
					theta = angle_betw(p1,p2,p3);	
				}
				////////如果两段轨迹间的夹角足够大，那么该段轨迹为过渡段
				if(theta>0&&(myspace_traj[jj].isinverse==myspace_traj[jj+1].isinverse)) 
				myspace_traj[jj].traj_type = TRAJ_MIDDLE;
				//////否则定义该段类型为结束段
				else 
				myspace_traj[jj].traj_type = TRAJ_END;
			}
		}
	}


	for(int tmii = 0;tmii<myspace_traj.size();tmii++)
	printf("收到轨迹参数，spx=%f,spy=%f>>>epx=%f,epy=%f,trajtype=%d,linetype=%d,ini_u=%d,end_u=%d,slow_u=%d,total-u=%d,length=%f,inverse=%d,ending=%d\n",myspace_traj[tmii].tarjp.CURVE_INITIAL_POSE.x,myspace_traj[tmii].tarjp.CURVE_INITIAL_POSE.y,myspace_traj[tmii].tarjp.CURVE_TARGET_POSE.x,myspace_traj[tmii].tarjp.CURVE_TARGET_POSE.y,myspace_traj[tmii].traj_type,myspace_traj[tmii].traj_type,myspace_traj[tmii].tarjp.ST_LKAHP_u,myspace_traj[tmii].tarjp.ED_LKAHP_u,myspace_traj[tmii].tarjp.slow_u,myspace_traj[tmii].tarjp.total_u_num,myspace_traj[tmii].tarjp.length,myspace_traj[tmii].isinverse,myspace_traj[tmii].trisend);
	move_flag = do_act;
  LOOKAHEADFNISED = true;
  return TRAJ_LOOKAHEAD_OK;//////路径前瞻完成 
}
double curve_run::angle_betw(Position pos1,Position pos2,Position pos3)/////三点间的夹角
{//////如果三点夹角大于150,则不减速
	Position vet1;
	Position vet2;
	vet1 = (pos2-pos1);
	vet2 = (pos2-pos3);
	double den,cos1,num,arccos;
	double L0x,L0y,L1x,L1y;
	L0x = vet1.x;L0y = vet1.y;
	L1x = vet2.x;L1y = vet2.y;
	num = (L0x*L1x + L0y*L1y );//分子
	den = (sqrt(L0x*L0x + L0y*L0y))*(sqrt(L1x*L1x + L1y*L1y));//分母
	cos1 = num/den;
	if(cos1<=-1) 
		cos1=-1;
	if(cos1>=1)
		cos1=1;
	arccos = acos(cos1)/M_PI*180;//计算角度，由于计算出来的是弧度，所以划算成度了
	if(fabs(arccos)>170) 
		return 1.0;
	else	return -1.0;
}
void * curve_run::curve_move_thread_(void * param)
{
  curve_run *ptr = (curve_run *)param;
  int t_acc,t_dec,t_even,t_total;
	double l_acc,l_dec,l_even,l_total;
	char motion_type;
	char run_step;
	int t_cur;
	struct timeval last;
	gettimeofday(&last,NULL);
	long last_time=last.tv_sec*1000000+last.tv_usec;
	long cycle_100=last_time;
	Speed_data speed_send(0,0);
	while(ptr->task_status != ROBOT_TASK_STS_COMPLETE)
	{
    ptr->realpose_map = get_global_agv_instance()->expolate_current_position();
    ptr->CURVE_THREAD_RUN(cycle_100,t_cur,t_acc,t_dec,t_even,l_even,l_acc,l_dec,l_total,motion_type,run_step);
    ptr->jh_vel_test(ptr->auto_run_lvel, ptr->auto_run_rvel);
    speed_send.vel_line = ptr->auto_run_lvel;
    speed_send.vel_ang = ptr->auto_run_rvel;
    std::vector<char> tmp;
    speed_send.to_char_array(&tmp);
	if(ptr->mode_manual_auto == MODE_AUTO)
    ptr->queue->send(CHANNEL_MOTOR_CTL, tmp.data(), tmp.size());
    usleep(20000);
	}
	speed_send.vel_line = 0.0;
	speed_send.vel_ang = 0.0;
	std::vector<char> tmp;
  speed_send.to_char_array(&tmp);
	ptr->queue->send(CHANNEL_MOTOR_CTL, tmp.data(), tmp.size());
  	ptr->TRAJDONE = true;
	usleep(500000);
  int cmd = 0xc101;
  ptr->queue->send(CHANNEL_ARRIVE, (char *)(&cmd), sizeof(int));
	ptr->task_status = ROBOT_TASK_STS_NONE;

}
bool curve_run::dis_among(Position pos1,Position pos2,double dis)/////两个空间点之间的距离
{
	double dis_ret;
	double dis_x = pos1.x- pos2.x;
	if(fabs(dis_x)>dis) return false;
	double dis_y = pos1.y- pos2.y;
	if(fabs(dis_y)>dis) return false;
	
	double v02 = (dis_x)*(dis_x);
	double v12 = (dis_y)*(dis_y);
	dis_ret = sqrt(v02+v12);
	if(dis_ret>dis)
	return false;
	else
	return true;
}

bool curve_run::dis_between(Position pos1,Position pos2,double &dis)/////两个空间点之间的距离
{
	double dis_x = pos1.x- pos2.x;
	double dis_y = pos1.y- pos2.y;	
	double v02 = (dis_x)*(dis_x);
	double v12 = (dis_y)*(dis_y);
	
	dis = sqrt(v02+v12);	
}
Position curve_run::cal_CZ(Position pos0,Position pos1,Position pos2,int &dir)////计算垂足点
{
	double l0,l1,l2,tt;
	Position vet;
	Position vet1;


	Position vet2;
	double den,cos1,num,arccos;
	double L0x,L0y,L1x,L1y;

	l0 = sqsum(pos1,pos0);
	l1 = sqsum(pos2,pos0);
	l2 = sqsum(pos2,pos1);
	if((l0+l1-l2)<0.0001) vet=pos2;////三点共线，则直接返回该点
	else
	{
	tt=(l0+l1-l2)/(2*l0);
	vet.x = (1-tt)*pos0.x+tt*pos1.x;
	vet.y = (1-tt)*pos0.y+tt*pos1.y;
	vet.theta = pos2.theta;
	}
	
	vet1 = pos1-pos0;
	vet2 = pos1-vet;
	
	L0x = vet1.x;L0y = vet1.y;
	L1x = vet2.x;L1y = vet2.y;
	num = (L0x*L1x + L0y*L1y );//分子
	den = (sqrt(L0x*L0x + L0y*L0y))*(sqrt(L1x*L1x + L1y*L1y));//分母
	cos1 = num/den;
	if(cos1<=-1) 
		cos1=-1;
	if(cos1>=1)
		cos1=1;
	arccos = acos(cos1);//计算角度，由于vc计算出来的是弧度，所以划算成度了
	if(fabs(arccos)>M_PI*0.5) dir = -1;else dir = 1;

	return vet;
}
Position curve_run::curp2curve(Position pcur,T_CURVE_P &curtraj,int curvetype)////计算由当前点到曲线上距离最近的点
{
	Position pp;
	int init;	
	pp = find_initial_point( pcur, init, curtraj, curvetype);//////计算起始点位置
	//curtraj.init_u = init;
	return pp;
}
Position curve_run::find_initial_point( Position pcur,int &init_i, T_CURVE_P &curtraj,int curvetype)/////求解bezier曲线上的起始点坐标
{
	
	int ttotal = curtraj.total_u_num;
	Position p0, p1,  p2, p3,point,point1;
	
	p0 = curtraj.CURVE_INITIAL_POSE;

	p1 = curtraj.CURVE_CONTROLP1;
	p2 = curtraj.CURVE_CONTROLP2;
	p3 = curtraj.CURVE_TARGET_POSE;
	Position chuizup1;///垂足点
	Position chuizup2;///垂足点
	Position chuizup3;///垂足点
	Position chuizup;///垂足点
	int dirl1 =1, dirl2 =1, dirl3 = 1;
	double length,length1;
	double temp_len,target_theta;
	if(curvetype == TRAJ_ISCURVE)
	{
	chuizup1 = cal_CZ(p0,p3,pcur,dirl1);
	chuizup1 = cal_CZ(p3,p0,pcur,dirl2);
	if (dirl1<0||dirl2<0) 
	{
	if((sqsum(pcur,p0))<(sqsum(pcur,p3)))
	chuizup1 = p0;
	else
	chuizup1 = p3;
	}
	
	chuizup = chuizup1;
	printf("锤足位置%f-%f-%f\n",chuizup.x,chuizup.y,chuizup.theta);
	printf("P0位置%f-%f-%f\n",p0.x,p0.y,p0.theta);
	////////////////////找到离PCUR最近的垂足点

	}////////////////if(curvetype == TRAJ_ISCURVE)
	else
	{
	chuizup = cal_CZ(p0,p3,pcur,dirl1);
	chuizup = cal_CZ(p3,p0,pcur,dirl2);
	if (dirl1<0||dirl2<0) 
	chuizup = p0;
	printf("锤足位置%f-%f-%f\n",chuizup.x,chuizup.y,chuizup.theta);
	printf("P0位置%f-%f-%f\n",p0.x,p0.y,p0.theta);
	
	}
	//////////////////////////////////////判断锤足与轨迹起始点的距离，若足够小，则认为从起始点开始启动轨迹
	point = chuizup;
	temp_len = sqrt(sqsum(chuizup,p0));
	if(temp_len<0.5) 
	{init_i = 0;
	curtraj.init_u = init_i;
	printf("++++++++++++++ini_u=%d,ini_next_u=%d,total_u=%d\n",curtraj.init_u,curtraj.init_next_u,curtraj.total_u_num);
	if(curvetype == TRAJ_ISCURVE)
	point1 = BezierPoint((double)curtraj.ST_LKAHP_u / (double)ttotal,curtraj);
	else
	point1 = curtraj.CURVE_TARGET_POSE;
	point.theta = atan2(point1.y-point.y,point1.x-point.x);//-M_PI*0.5;//20191009更改-M_PI*0.5
	printf("point位置%f-%f-%f\n",point.x,point.y,point.theta);
	 return point;
   }
	
	if(curvetype == TRAJ_ISCURVE)
	{
	//////////////////////////////遍历一遍曲线上的点，找出曲线上离该垂足最近的点
	for (int i = 1; i <= ttotal; i++)
	{
	point = BezierPoint((double)i / (double)ttotal,curtraj);
	length = sqrt(sqsum(chuizup,point));
	if(temp_len>length) 
	{
		temp_len = length;
		init_i = i;/////起始点
	}
	}
	curtraj.init_u = init_i;
	point = BezierPoint((double)init_i / (double)ttotal, curtraj);//////得到曲线上起点的位置
	//point = curtraj.CURVE_INITIAL_POSE;////取曲线的起点作为该段轨迹的终点
	///////////////////////////求解距离该起点，look_ahead_dis的下一个位置点，并计算两点之间方向角作为目标点的姿态角
	length1 = 0.0;
	point1 = point;
	Position lastPoint = point1;
	for (int i = init_i+1; i <= ttotal; i++)
	{
	point1 = BezierPoint((double)i / (double)ttotal,curtraj);
	length1 += sqrt(sqsum(lastPoint,point));
	lastPoint = point1;
	init_i = i;
	if(length1>=look_ahead_dis) 
		break;
	}
	curtraj.init_next_u = init_i;
	}

	if(curvetype == TRAJ_ISCURVE)
	point1 = BezierPoint((double)curtraj.init_next_u / (double)ttotal,curtraj);
	else
	{
	point1 = curtraj.CURVE_TARGET_POSE;
	point = chuizup;
	}
	point.theta = atan2(point1.y-point.y,point1.x-point.x);//-M_PI*0.5;//20191009更改-M_PI*0.5
	printf("point位置%f-%f-%f\n",point.x,point.y,point.theta);
	printf("/////////////////////////ini_u=%d,ini_next_u=%d,total_u=%d\n",curtraj.init_u,curtraj.init_next_u,curtraj.total_u_num);
	return point;


}


Position curve_run::BezierPoint(double t, T_CURVE_P curtraj)/////求解bezier曲线上的点
{
	Position p0, p1,  p2, p3;
	p0 = curtraj.CURVE_INITIAL_POSE;
	p1 = curtraj.CURVE_CONTROLP1;
	p2 = curtraj.CURVE_CONTROLP2;
	p3 = curtraj.CURVE_TARGET_POSE;
	double u = 1 - t;
	double tt = t * t;
	double uu = u * u;
	double uuu = uu * u;
	double ttt = tt * t;

	Position p;
	p.x = uuu * p0.x;
	p.y = uuu * p0.y;

	p.x += 3 * uu * t * p1.x;
	p.y += 3 * uu * t * p1.y;

	p.x += 3 * u * tt * p2.x;
	p.y += 3 * u * tt * p2.y;

	p.x += ttt * p3.x;
	p.y += ttt * p3.y;
	return p;
}
double curve_run::sqsum(Position pos0,Position pos1)
{
	double len,xx,yy;
	Position sub;
	sub = pos0-pos1;
	xx=sub.x*sub.x;
	yy=sub.y*sub.y;
	len = xx+yy;
	return len;
}
double curve_run::getBezier_length( T_CURVE_P &curtraj)////求解bezier曲线的长度
{
	int pointCount;
	Position p0, p1,  p2, p3;
	p0 = curtraj.CURVE_INITIAL_POSE;
	p1 = curtraj.CURVE_CONTROLP1;
	p2 = curtraj.CURVE_CONTROLP2;
	p3 = curtraj.CURVE_TARGET_POSE;
	Position point;
	double len1 = sqrt(sqsum(p0,p1));
	double len2 = sqrt(sqsum(p1,p2));
	double len3 = sqrt(sqsum(p2,p3));
	int start_i,end_i;
	bool start_find=false,end_find=false;
	pointCount = int((len1+len2+len3)*200);
	
	double length = 0.0;
	Position lastPoint = p0;
	printf("len1=%f,len2=%f,len3=%f,.......pointcount=%d\n",len1,len2,len3,pointCount);
	for (int i = 1; i <= pointCount; i++)
	{
	point = BezierPoint((double)i / (double)pointCount, curtraj);
	length += sqrt(sqsum(lastPoint,point));
	lastPoint = point;
	}
	pointCount = int((length)*200);/////5mm一个点
	curtraj.total_u_num = pointCount;///////曲线总u数目
	curtraj.length = length;//////曲线的总长度
///////////////////////////////////求解起始前瞻点处的前瞻点索引
	length = 0.0;
	lastPoint = p0;
	for (int k = 1; k <= pointCount; k++)
	{
	point = BezierPoint((double)k / (double)pointCount, curtraj);
	length += sqrt(sqsum(lastPoint,point));
	if(length>look_ahead_dis&&start_find==false)
	{start_i = k;
	printf("start_i=%d,length=%f,look_ahead_dis=%f\n",start_i,length,look_ahead_dis);
	start_find=true;
	break;}
	lastPoint = point;
	}
/////////////////////////////找出曲线上距离目标位置slow_distance的点对应的索引
	double length1 = 0.0;
	lastPoint = p3;
	for (int j = pointCount; j > 1; j--)
	{
	point = BezierPoint((double)j / (double)pointCount,curtraj);
	length1 += sqrt(sqsum(lastPoint,point));
	lastPoint = point;
	if(end_find==false&&length1>look_ahead_dis)
	{end_i = j;
	printf("end_i=%d\n",end_i);	
	end_find=true;}////找到结束点处的前瞻点索引
	if(length1>=nav_slow_dis) 
	{
		curtraj.slow_u = j;/////开始降速的点
		break;
	}
	}
/////////////////////////////////////////////////
	curtraj.ST_LKAHP = BezierPoint((double)start_i / (double)pointCount,curtraj);
	curtraj.ST_LKAHP_u =start_i;
	curtraj.ED_LKAHP = BezierPoint((double)end_i / (double)pointCount,curtraj);
	curtraj.ED_LKAHP_u =end_i;
	curtraj.init_u =0;
//////////////////////////////////////////////////
	return length;
}

bool curve_run::traj_par_init()
{
		i_step = 0;
		running_data.traj_index = 0;/////取第一条轨迹
		CURBEZIER = myspace_traj[running_data.traj_index];
		running_data.T_INIPRDY = false;////临时轨迹起始点READY
		running_data.T_ENDPRDY = false;////临时轨迹终止点READY
		running_data.T_NEXTPRDY = false;////临时轨迹下一点READY
		running_data.T_INIPID = 0;////临时轨迹起始点索引0
		running_data.T_ENDPID = 0;////临时轨迹终止点索引0
		running_data.T_NEXTPID = 0;////临时轨迹下一点索引0
		loadtrajpar();
}

void curve_run::loadtrajpar()/////加载轨迹参数
{
	if(LOOKAHEADFNISED != true) {printf("condition113 exit \n"); return;};//////如果前瞻未完成，则退出

	int ii;
	double total_len;
	T_CURVE_P curtraj;	
	double length1 = 0.0;
	int ttotal;
	Position point1;
	Position lastPoint;
	char step_temp;

	double length_temp_1;	
	if(CURBEZIER.PTP_OR_CURVE ==TRAJ_ISCURVE)
	{
	curtraj = CURBEZIER.tarjp;
	length1 = 0.0;
	ttotal = curtraj.total_u_num;
	length_temp_1 = look_ahead_dis;
	//////////////////////////////////计算临时轨迹参数
	if(running_data.T_INIPRDY == false)
	{////起始点未准备好，计算起点，终点，下一点
    TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
		if(curtraj.init_u<=1||curtraj.init_u>ttotal)
		{
		running_data.T_INIP = curtraj.CURVE_INITIAL_POSE;
		running_data.T_ENDP = curtraj.ST_LKAHP;
		running_data.T_INIPRDY = true;
		running_data.T_ENDPRDY = true;
		running_data.T_ENDPID = curtraj.ST_LKAHP_u;
		step_temp = 11;
		}
		else
		{
		running_data.T_INIP = BezierPoint((double)curtraj.init_u / (double)ttotal,curtraj);
		running_data.T_ENDP = BezierPoint((double)curtraj.init_next_u / (double)ttotal,curtraj);
		running_data.T_INIPRDY = true;
		running_data.T_ENDPRDY = true;
		running_data.T_ENDPID = curtraj.init_next_u;
		step_temp = 22;
		}
		length1 = 0.0;
		point1 = running_data.T_ENDP;
		lastPoint = point1;
		for ( ii = running_data.T_ENDPID+1; ii <= ttotal; ii++)
		{
		point1 = BezierPoint((double)ii / (double)ttotal,curtraj);
		length1 += sqrt(sqsum(lastPoint,point1));
		lastPoint = point1;
		if(length1>=length_temp_1) 
			break;
		}
		running_data.T_NEXTP = BezierPoint((double)ii / (double)ttotal,curtraj);
		running_data.T_NEXTPID = ii;
		running_data.T_NEXTPRDY = true;
		printf("起始运动点计step_temp%d算完成-------iniid=%d....endid=%d....nextid=%d>>LKAHP_u=%d\n",step_temp ,curtraj.init_u,running_data.T_ENDPID,running_data.T_NEXTPID,curtraj.ST_LKAHP_u);
	}
	else if(running_data.T_NEXTPRDY == false)
	{//////若仅仅是nextp未准备好，那么仅计算下一点的位置，索引
		if(running_data.T_NEXTPID>=ttotal) 
		{////如果当前已经取到轨迹的最终点，则不再计算
		running_data.T_NEXTPRDY = true;
		return;
		}
		length1 = 0.0;
		point1 = running_data.T_NEXTP;
		lastPoint = point1;
		for ( ii = running_data.T_NEXTPID+1; ii <= ttotal; ii++)
		{
		point1 = BezierPoint((double)ii / (double)ttotal,curtraj);
		length1 += sqrt(sqsum(lastPoint,point1));
		lastPoint = point1;
		if(length1>=length_temp_1) 
			break;
		}
		if(ii>=ttotal) 
		running_data.T_NEXTP = curtraj.CURVE_TARGET_POSE;
		else
		running_data.T_NEXTP = BezierPoint((double)ii / (double)ttotal,curtraj);
		running_data.T_NEXTPID = ii;
		running_data.T_NEXTPRDY = true;
		//printf("过渡运动点计算完成==============\n");
		/////////如果当前轨段是起始或者过渡段，那么就不会进入减速过程
		if(CURBEZIER.traj_type == TRAJ_START||CURBEZIER.traj_type == TRAJ_MIDDLE)
		{
			if(running_data.T_NEXTPID>=curtraj.ED_LKAHP_u)/////判断是否到达最终前瞻距离阶段
			{
				running_data.T_NEXTP = curtraj.CURVE_TARGET_POSE;////BezierPoint((double)curtraj.slow_u / (double)ttotal,curtraj);
				running_data.T_NEXTPID = curtraj.total_u_num;
				running_data.T_NEXTPRDY = true;

				dvect_between(curtraj.CURVE_TARGET_POSE,curtraj.ED_LKAHP,Traj.PTP_TARGET_DIR,total_len);
		running_data.T_INIP = curtraj.ED_LKAHP;
		running_data.T_ENDP = curtraj.CURVE_TARGET_POSE;
		//printf("切换方向矢量，过渡段最后一个点T_TOTAL=%d====T_INIPID=%d======T_ENDPID=%d=\n,dirx=%f,diry=%f ",CURBEZIER.tarjp.total_u_num,running_data.T_INIPID,running_data.T_ENDPID,Traj.PTP_TARGET_DIR.x,Traj.PTP_TARGET_DIR.y);
				printf("匀速阶段结束运动点+++++++++++++计算完成\n");
			}
			
		}
		else///////如果当前轨迹段是结束或者完整段，那么会进入减速阶段
		if(running_data.T_NEXTPID>=curtraj.slow_u)/////判断是否进入减速阶段
		{
			if(running_data.T_NEXTPID>=curtraj.ED_LKAHP_u)/////判断是否到达最终前瞻距离阶段
			{
				running_data.T_NEXTP = curtraj.CURVE_TARGET_POSE;////BezierPoint((double)curtraj.slow_u / (double)ttotal,curtraj);
				running_data.T_NEXTPID = curtraj.total_u_num;
				running_data.T_NEXTPRDY = true;
				printf("减速阶段结束运动点+++++++++++++计算完成");
			}
			if(TRAJ_RUNNING_STEP != TRAJ_RUN_STEPDEC)///;/////切换至减速阶段
			{
				TRAJ_RUNNING_STEP = TRAJ_RUN_STEPDEC;
				running_time = 0;/////将时间置位
			}
			//printf("结束运动点计算完成==============\n");
		}
	}
	task_status = ROBOT_TASK_STS_RUNNING;
	}
}


double  curve_run::jh_sign(double in)
{
	double out;
	if(in>0) out = 1.0;
	else if(in<0) out = -1.0;
	return out;
}
bool curve_run::dvect_between(Position pos1,Position pos2,Position &dvector,double &l_tlen)/////两个空间点之间的单位方向矢量
{
	Position vec_bet;
	double l_total;
	vec_bet = (pos1 - pos2);
	l_total = sqrt(vec_bet.x*vec_bet.x+vec_bet.y*vec_bet.y);
	vec_bet.x = vec_bet.x/l_total;
	vec_bet.y = vec_bet.y/l_total;
	vec_bet.theta = 0;
	dvector = vec_bet;
	//printf("当前轨迹的方向矢量x=%f,y=%f\n",dvector.x,dvector.y);
	l_tlen = l_total;
}

void  curve_run::CURVE_THREAD_RUN(long &pre_time,int &t_cur,int &t_acc,int &t_dec,int &t_even,double &l_even,double &l_acc,double &l_dec,double &l_total,char &motion_type,char &runstep)
{/////////////
	
	double total_len;
	double vref,wref,delta_t;
	Position retdeltap;
	Position JB_deltap;//////////机器人局部坐标系下的偏移量
	Position dis_target;///////距离目标点的位姿差值
	double theta,stheta,ctheta;

	int neednp;
	double direct_inverse=0.0;///////轨迹运行方向是否反向
	bool bjump = false;
	struct timeval current;
	gettimeofday(&current,NULL);
	long current_time=current.tv_sec*1000000+current.tv_usec;
	delta_t=double(current_time-pre_time)/1000;
	double delta_theta_temp;
	//if(delta_t>=(CONTROL_CYCLE_TIME))
	{
		pre_time=current_time;
		if(LOOKAHEADFNISED != true||(mode_manual_auto != MODE_AUTO)) //return;//////如果前瞻未完成，则退出
		{
			vel_start = auto_run_lvel;
			//auto_run_lvel = 0.0;
			//auto_run_rvel = 0.0;
			//printf("LOOKAHEADFNIS != true，则不执行任务\n");
			return;
		}

		if(CURBEZIER.PTP_OR_CURVE == TRAJ_ISCURVE||CURBEZIER.PTP_OR_CURVE == TRAJ_ISPTP||CURBEZIER.PTP_OR_CURVE == TRAJ_ISFREE)
		{
			//printf("运行BEZIER曲线运动程序》AAAAAAAAAAAAAAAAAAAAAAAAAAAA》\n");
			if(i_step == 0)////////////////启动步骤0旋转规划步骤
			{///
			if(dis_among(realpose_map,CURBEZIER.tarjp.CURVE_TARGET_POSE,0.1))
			{////////////直线步骤定位运动完成
			printf("当前位置距离目标点小于一定距离，机器人停止\n");
			running_data.T_ENDPID = CURBEZIER.tarjp.total_u_num;///将当前运行参数置为
			////////////////////////////////
			if(CURBEZIER.trisend !=TRAJ_ISENDDING)///并且不是最终结束段
			{////////////直线定位运动完成,跳至步骤4
				nexttraj();////切换下一段轨迹
				i_step = 0;////从下一段的旋转阶段开始运动
				auto_run_lvel = 0.0;
				auto_run_rvel = 0.0;
				vel_start = auto_run_lvel;
				TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
				running_time = 0;//////运动计时清零
				printf("切换至下一段轨迹路径\n");
				return;
			}
			else
			{
				i_step = 4;////从当前段的旋转阶段开始运动
				auto_run_lvel = 0.0;
				auto_run_rvel = 0.0;
				vel_start = auto_run_lvel;
				TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
				running_time = 0;//////运动计时清零
				return;
			}
			
			}
////////////////////////////////////////////////////////////////////////////////////////
			////////////如果inip和endp都未准备好，则返回
			if(CURBEZIER.isinverse == true)
			direct_inverse = 1.0;
			if(CURBEZIER.PTP_OR_CURVE==TRAJ_ISCURVE)
			{//////如果当前轨迹是曲线
			//printf("========777777777777777777777777777777777===========================\n");
			if(running_data.T_INIPRDY != true||running_data.T_ENDPRDY != true) {loadtrajpar();return;}
			targetangle = atan2(CURBEZIER.tarjp.ST_LKAHP.y-CURBEZIER.tarjp.CURVE_INITIAL_POSE.y,CURBEZIER.tarjp.ST_LKAHP.x-CURBEZIER.tarjp.CURVE_INITIAL_POSE.x)-M_PI*(direct_inverse);//////计算目标姿态角度//20191009更改-M_PI*(0.5+direct_inverse);
			printf("=======================================================================\n");
			printf("-------目标角度=%f\n",targetangle);
			printf("=======================================================================\n");
			}
			
			else if(CURBEZIER.PTP_OR_CURVE==TRAJ_ISFREE)
			{
			if(running_data.T_INIPRDY != true||running_data.T_ENDPRDY != true) {loadtrajpar();return;}
			//targetangle = atan2(my_pathplan[4].y-my_pathplan[0].y,my_pathplan[4].x-my_pathplan[0].x);
			//targetangle = atan2(running_data.T_ENDP.y-running_data.T_INIP.y,running_data.T_ENDP.x-running_data.T_INIP.x)-M_PI*0.5;//////计算目标姿态角度
			printf("=======================================================================\n");
			printf("----5555555目标角度=%f\n",targetangle);
			printf("=======================================================================\n");			
			}
			else//////20200523目前取消了直线，对应的有自由曲线，如果是自由曲线那么采用自由曲线的第5个点，作为起始姿态角度的计算点
			targetangle = atan2(CURBEZIER.tarjp.CURVE_TARGET_POSE.y-realpose_map.y,CURBEZIER.tarjp.CURVE_TARGET_POSE.x-realpose_map.x)-M_PI*(direct_inverse);//20191009更改-M_PI*(0.5+direct_inverse);

			if(targetangle>M_PI)	targetangle-=2*M_PI;
			else if(targetangle<-M_PI)	targetangle+=2*M_PI;
			
			total_len = targetangle-realpose_map.theta;	
			
			if(total_len>M_PI)	total_len-=2*M_PI;
			else if(total_len<-M_PI)	total_len+=2*M_PI;

			if(fabs(total_len)<allow_angle)///////第一步旋转
			{////////////第一步旋转定位运动完成,跳至步骤2
			i_step = 2;
			auto_run_lvel = 0.0;
			auto_run_rvel = 0.0;
			vel_start = auto_run_lvel;
			TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识

			running_time = 0;//////运动计时清零
			printf("程序跳过，直接运行步骤2222222\n");
			return;
			}
			JH_LEN2T(fabs(total_len),1,t_acc,t_dec,t_even,l_even,l_acc,l_dec,l_total,motion_type);/////////规划运行轨迹
			
			///////////////////计算方向矢量
			Traj.PTP_TARGET_DIR.x = 0;
			Traj.PTP_TARGET_DIR.y = 0;
			Traj.PTP_TARGET_DIR.theta = jh_sign(total_len);
			i_step = 1;
			
			//printf("target_angle = %f,realpose_map.theta=%f,total_len=%f,DIR.theta=%f\n",targetangle,realpose_map.theta,total_len,Traj.PTP_TARGET_DIR.theta);
			printf("运行点到点程序==步骤-tacc=%d,t_dec=%d,t_even=%d,lacc=%f,ldec=%f,ltotal=%f,motiontype=%d===\n",t_acc,t_dec,t_even,l_acc,l_dec,l_total,motion_type);
			running_time = 0;//////运动计时清零
			t_cur=0;
			}//////if(i_step == 0)////////////////启动步骤0
			else if(i_step == 1)////////////////启动步骤1旋转步骤
			{/////////运行旋转步骤
				delta_theta_temp = targetangle-realpose_map.theta;	
				if(delta_theta_temp>M_PI)	delta_theta_temp-=2*M_PI;
				else if(delta_theta_temp<-M_PI)	delta_theta_temp+=2*M_PI;

				retdeltap = cal_deltap_(TRAJ_ROTATE,delta_t,t_cur,t_acc,t_dec,t_even,l_even,l_acc,l_dec,l_total,motion_type,vref,wref,neednp);
				dis_target.theta = targetangle-realpose_map.theta;
				if((fabs(dis_target.theta)<allow_angle)||(fabs(dis_target.theta+M_PI*2)<allow_angle)||(fabs(dis_target.theta-M_PI*2)<allow_angle)||((jh_sign(delta_theta_temp)==-Traj.PTP_TARGET_DIR.theta)&&(TRAJ_RUNNING_STEP == TRAJ_RUN_STEPDEC)))///第一步旋转到目标姿态
				{////////////第一步旋转定位运动完成
				i_step = 2;
				runstep = 2;
				auto_run_lvel = 0.0;
				auto_run_rvel = 0.0;
				vel_start = auto_run_lvel;
				TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
				running_time = 0;//////运动计时清零
				return;
				}
				else//////////////////////////
				{////////////////////////将偏差数据转换到机器人局部坐标系下
				JB_deltap.x = retdeltap.x;
				JB_deltap.y = retdeltap.y;
				JB_deltap.theta = retdeltap.theta;
				/////////////////////////////李雅普诺夫函数计算系统输出
				line_vel = vref*cos(JB_deltap.theta) + LYP_KX*JB_deltap.x;
				angle_vel = wref+vref*LYP_KY*JB_deltap.y+LYP_KPHY*sin(JB_deltap.theta);
				//jh_vel_test( line_vel, angle_vel);
				/////////////////////////////逆解到左右轮的目标转速
				if (Traj.PTP_TARGET_DIR.theta>0)
				{
				auto_run_lvel = line_vel;
				auto_run_rvel = angle_vel;
				}
				else
				{
				auto_run_lvel = line_vel;
				auto_run_rvel = -angle_vel;
				}

				}
			}//////if(i_step == 1)////////////////启动步骤1
			else if(i_step == 2)////////////////启动步骤2直线规划阶段
			{////////////规划第二步直线运行阶段步骤
				if(CURBEZIER.PTP_OR_CURVE==TRAJ_ISCURVE||(CURBEZIER.PTP_OR_CURVE==TRAJ_ISFREE))
				{//////如果当前轨迹是曲线
				if(running_data.T_INIPRDY != true||running_data.T_ENDPRDY != true) {loadtrajpar();return;}
				dvect_between(running_data.T_ENDP,running_data.T_INIP,Traj.PTP_TARGET_DIR,total_len);
				//printf("ENDP.x=%f,ENDP.y=%f\n",running_data.T_ENDP.x,running_data.T_ENDP.y);
				//printf("INIP.x=%f,INIP.y=%f\n",running_data.T_INIP.x,running_data.T_INIP.y);
				//printf("PTP_TARGET_DIR.x=%f,PTP_TARGET_DIR.y=%f\n",Traj.PTP_TARGET_DIR.x,Traj.PTP_TARGET_DIR.y);
				}
				else //////如果当前轨迹是直线
				dvect_between(CURBEZIER.tarjp.CURVE_TARGET_POSE, CURBEZIER.tarjp.CURVE_INITIAL_POSE, Traj.PTP_TARGET_DIR,total_len);
				dis_between(CURBEZIER.tarjp.CURVE_TARGET_POSE,realpose_map,total_len);
		
				targetangle = atan2(Traj.PTP_TARGET_DIR.y,Traj.PTP_TARGET_DIR.x);
				if(CURBEZIER.traj_type == TRAJ_START||CURBEZIER.traj_type == TRAJ_MIDDLE)////如果当前轨迹不是ENDING段
				{//////如果是起始或者过渡段，则只要小于pass_allow_dis,就认为目标到达
					if(fabs(total_len)<pass_allow_dis)
					{////////////直线定位运动完成,跳至步骤4
					nexttraj();////切换下一段轨迹
					vel_start = auto_run_lvel;
					i_step = 2;////从直线阶段开始运动
					runstep = 2;
					TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
					running_time = 0;//////运动计时清零
					return;
					}
				}
				else ////如果当前轨迹类型是完整或终止段，
				{
					if(CURBEZIER.trisend !=TRAJ_ISENDDING)///并且不是最终结束段
					{	if(fabs(total_len)<allow_distance)
						{////////////直线定位运动完成,跳至步骤0
							nexttraj();////切换下一段轨迹
							auto_run_lvel = 0.0;
							auto_run_rvel = 0.0;
							i_step = 0;////从下一段的旋转阶段开始运动
							runstep = 0;
							printf("原因888导致停止\n");
							vel_start = auto_run_lvel;
							TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
							running_time = 0;//////运动计时清零
							return;
						}
					}
					else/////是最终的结束段
					{
						if(fabs(total_len)<allow_distance)
						{////////////直线定位运动完成,跳至步骤4
							////nexttraj();////不再切换下一段轨迹
							i_step = 4;////从当前段的旋转阶段开始运动
							runstep = 4;
							auto_run_lvel = 0.0;
							auto_run_rvel = 0.0;
							vel_start = auto_run_lvel;
							TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
							running_time = 0;//////运动计时清零
							return;
						}
					}////else/////是最终的结束段
				}
				total_len = CURBEZIER.tarjp.length;
				JH_LEN2T(fabs(total_len),0,t_acc,t_dec,t_even,l_even,l_acc,l_dec,l_total,motion_type);/////////规划运行轨迹
				i_step = 3;
				target_reached_count = 0;
				printf("运行点到点程序>>>>步骤22222222>>>>>\n");
				t_cur=0;
				running_time = 0;//////运动计时清零
				return;
			}/////else if(i_step == 2)////////////////启动步骤2
			else if(i_step == 3)////////////////启动步骤3
			{////////////启动直线运动步骤
				if(CURBEZIER.PTP_OR_CURVE==TRAJ_ISCURVE||(CURBEZIER.PTP_OR_CURVE==TRAJ_ISFREE))
				{//////如果当前轨迹是曲线
				if(running_data.T_INIPRDY != true||running_data.T_ENDPRDY != true) return;
				}
				t_cur++;
				retdeltap = cal_deltap_(TRAJ_LINE,delta_t,t_cur,t_acc,t_dec,t_even,l_even,l_acc,l_dec,l_total,motion_type,vref,wref,neednp);
				
				{////////////////////////将偏差数据转换到机器人局部坐标系下
				JB_deltap.x = retdeltap.x;
				JB_deltap.y = retdeltap.y;
				JB_deltap.theta = retdeltap.theta;
				/////////////////////////////李雅普诺夫函数计算系统输出
				line_vel = (vref*cos(JB_deltap.theta) + LYP_KX*JB_deltap.x)*LYP_TTP;
				angle_vel = (wref+vref*LYP_KY*JB_deltap.y+LYP_KPHY*sin(JB_deltap.theta))*LYP_TTP;			
				//jh_vel_test( line_vel, angle_vel);
				/////////////////////////////逆解到左右轮的目标转速
				auto_run_lvel = line_vel;
				auto_run_rvel = angle_vel;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////轨迹运行终点判读条件
				if(CURBEZIER.traj_type == TRAJ_START||CURBEZIER.traj_type == TRAJ_MIDDLE)////
				{//////如果是起始或者过渡段，则只要小于pass_allow_dis,就认为目标到达
					if(running_data.T_ENDPID >= CURBEZIER.tarjp.total_u_num)
					{////////////直线定位运动完成,跳至步骤4
					nexttraj();////切换下一段轨迹
					vel_start = auto_run_lvel;
					i_step = 2;////从直线阶段开始运动
					runstep = 2;
					TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
					running_time = 0;//////运动计时清零
					printf("过渡段，目标点在机器人正前方满足位置到达条件\n");
					neednp = TRAJ_NEED_NEXTP_N;////取消取下一点的操作
					return;
					}
				}
				else ////如果当前轨迹类型是完整或终止段，
				{
					if(CURBEZIER.trisend !=TRAJ_ISENDDING)///并且不是最终结束段
					{
						if(CURBEZIER.PTP_OR_CURVE == TRAJ_ISCURVE||(CURBEZIER.PTP_OR_CURVE == TRAJ_ISFREE))/////如果当i前轨迹是曲线
						if(((fabs(JB_deltap.x)<allow_distance||JB_deltap.x<0) && running_data.T_ENDPID>=CURBEZIER.tarjp.total_u_num))//||(target_reached_count>10))
						bjump = true;
						if(CURBEZIER.PTP_OR_CURVE == TRAJ_ISPTP)/////如果当i前轨迹是直线
						if(fabs(JB_deltap.x)<allow_distance) bjump = true;
						if(bjump)
						{////////////直线定位运动完成,跳至步骤4
							nexttraj();////切换下一段轨迹
							i_step = 0;////从下一段的旋转阶段开始运动
							runstep = 0;
							printf("原因111导致停止\n");
							auto_run_lvel = 0.0;
							auto_run_rvel = 0.0;
							vel_start = auto_run_lvel;
							TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
							running_time = 0;//////运动计时清零
							printf("完整段，目标点在机器人正前方满足位置到达条件\n");
							return;
						}
					}
					else/////是最终的结束段
					{
						if(CURBEZIER.PTP_OR_CURVE == TRAJ_ISCURVE||(CURBEZIER.PTP_OR_CURVE == TRAJ_ISFREE))/////如果当i前轨迹是曲线
						if(((fabs(JB_deltap.x)<allow_distance||JB_deltap.x<0) && running_data.T_ENDPID>=CURBEZIER.tarjp.total_u_num))//||(target_reached_count>10))//||(neednp == TRAJ_NEED_NEXTP_Y && running_data.T_INIPID>=CURBEZIER.tarjp.total_u_num))
						bjump = true;
						if(CURBEZIER.PTP_OR_CURVE == TRAJ_ISPTP)/////如果当i前轨迹是直线
						if(fabs(JB_deltap.x)<allow_distance) bjump = true;
						if(bjump)
						{////////////直线定位运动完成,跳至步骤4
							////nexttraj();////不再切换下一段轨迹
							i_step = 4;////从当前段的旋转阶段开始运动
							runstep = 4;
							auto_run_lvel = 0.0;
							auto_run_rvel = 0.0;
							vel_start = auto_run_lvel;
							TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
							running_time = 0;//////运动计时清零
	//printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
	//printf("终止段ENDDING，目标点在机器人正前方满足位置到达条件,x=%f,y=%ftheta=%f,\n",realpose_map.x,realpose_map.y,realpose_map.theta);
	//printf("机器人距离目标点,delta x=%f,delta_y=%fdelta_theta=%f,wref=%f,\n",JB_deltap.x,JB_deltap.y,JB_deltap.theta,wref);
	//printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
							return;
						}
					}////else/////是最终的结束段
				}
				}
				if(neednp == TRAJ_NEED_NEXTP_Y) exchangetraj();////取下一段路径参数
				//printf("当前周期的先速度=%f,角速度=%f>>>>左轮速度%f--%f\n",line_vel,angle_vel,leftwheelvel,rightwheelvel);
			}/////else if(i_step == 3)////////////////启动步骤3
			else if(i_step == 4)////////////////启动步骤4旋转规划
			{////////////规划第三步旋转动作
				targetangle = CURBEZIER.tarjp.CURVE_TARGET_POSE.theta;
				if(targetangle>M_PI)	targetangle-=2*M_PI;
				else if(targetangle<-M_PI)	targetangle+=2*M_PI;

				total_len = targetangle-realpose_map.theta;

				if(total_len>M_PI)	total_len-=2*M_PI;
				else if(total_len<-M_PI)	total_len+=2*M_PI;

				if(CURBEZIER.trisend !=TRAJ_ISENDDING)///并且不是最终结束段
				{	if((fabs(total_len)<allow_angle)||(fabs(total_len+M_PI*2)<allow_angle)||(fabs(total_len-M_PI*2)<allow_angle))
					{////////////
						nexttraj();////切换下一段轨迹
						i_step = 0;////从下一段的旋转阶段开始运动
						runstep = 0;
						printf("原因222导致停止\n");
						auto_run_lvel = 0.0;
						auto_run_rvel = 0.0;
					  vel_start = auto_run_lvel;
						TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
						running_time = 0;//////运动计时清零
						return;
					}
				}
				else/////是最终的结束段
				{
					if((fabs(total_len)<allow_angle)||(fabs(total_len+M_PI*2)<allow_angle)||(fabs(total_len-M_PI*2)<allow_angle))
					{////////////
						i_step = 6;////从当前段的旋转阶段开始运动
						runstep = 6;
						printf("原因333导致停止\n");
						auto_run_lvel = 0.0;
						auto_run_rvel = 0.0;
					  	vel_start = auto_run_lvel;
						TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
						running_time = 0;//////运动计时清零
						/////////////////////////
						//task_status = ROBOT_TASK_STS_COMPLETE;
						
						return;
					}
				}////else/////是最终的结束段

				JH_LEN2T(fabs(total_len),1,t_acc,t_dec,t_even,l_even,l_acc,l_dec,l_total,motion_type);/////////规划旋转运行轨迹
				///////////////////计算方向矢量
				Traj.PTP_TARGET_DIR.x = 0;
				Traj.PTP_TARGET_DIR.y = 0;
				Traj.PTP_TARGET_DIR.theta = jh_sign(total_len);
				i_step = 5;
				runstep = 5;
				printf("运行点到点程序》》》步骤4444444》》》》\n");
				t_cur=0;
				running_time = 0;//////运动计时清零
			}////else if(i_step == 4)////////////////启动步骤4旋转规划
			else if(i_step == 5)////////////////启动步骤5旋转运动
			{////////////启动旋转运动步骤
				t_cur++;
				retdeltap = cal_deltap_(TRAJ_ROTATE,delta_t,t_cur,t_acc,t_dec,t_even,l_even,l_acc,l_dec,l_total,motion_type,vref,wref,neednp);
				dis_target  = (CURBEZIER.tarjp.CURVE_TARGET_POSE - realpose_map);
				{////////////////////////将偏差数据转换到机器人局部坐标系下
				JB_deltap.x = retdeltap.x;
				JB_deltap.y = retdeltap.y;
				JB_deltap.theta = retdeltap.theta;

				line_vel = vref*cos(JB_deltap.theta) + LYP_KX*JB_deltap.x;
				angle_vel = wref+vref*LYP_KY*JB_deltap.y+LYP_KPHY*sin(JB_deltap.theta);
				//jh_vel_test( line_vel, angle_vel);
				/////////////////////////////逆解到左右轮的目标转速
				if (Traj.PTP_TARGET_DIR.theta>0)
				{
				auto_run_lvel = line_vel;
				auto_run_rvel = angle_vel;
				}
				else
				{
				auto_run_lvel = line_vel;
				auto_run_rvel = -angle_vel;
				}
				}
				delta_theta_temp = targetangle-realpose_map.theta;	
				if(delta_theta_temp>M_PI)	delta_theta_temp-=2*M_PI;
				else if(delta_theta_temp<-M_PI)	delta_theta_temp+=2*M_PI;
				if(CURBEZIER.trisend !=TRAJ_ISENDDING)///并且不是最终结束段
				{	if((fabs(dis_target.theta)<allow_angle)||(fabs(dis_target.theta+M_PI*2)<allow_angle)||(fabs(dis_target.theta-M_PI*2)<allow_angle)||((jh_sign(delta_theta_temp)==-Traj.PTP_TARGET_DIR.theta)&&(TRAJ_RUNNING_STEP == TRAJ_RUN_STEPDEC)))
					{////////////
						nexttraj();////切换下一段轨迹
						i_step = 0;////从下一段的旋转阶段开始运动
						runstep = 0;
						printf("原因4444导致停止\n");
						auto_run_lvel = 0.0;
						auto_run_rvel = 0.0;
						vel_start = auto_run_lvel;
						TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
						running_time = 0;//////运动计时清零
						return;
					}
				}
				else/////是最终的结束段
				{
					if((fabs(dis_target.theta)<allow_angle)||(fabs(dis_target.theta+M_PI*2)<allow_angle)||(fabs(dis_target.theta-M_PI*2)<allow_angle)||((jh_sign(delta_theta_temp)==-Traj.PTP_TARGET_DIR.theta)&&(TRAJ_RUNNING_STEP == TRAJ_RUN_STEPDEC)))
					{////////////
						////nexttraj();////不再切换下一段轨迹
						i_step = 6;////从当前段的旋转阶段开始运动
						runstep = 6;
						printf("原因555导致停止\n");
						auto_run_lvel = 0.0;
						auto_run_rvel = 0.0;
						vel_start = auto_run_lvel;
						/////////////////////////
						TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
						running_time = 0;//////运动计时清零
						printf("旋转定位完成》》》步骤5555555》》,x=%f,y=%ftheta=%f,\n",realpose_map.x,realpose_map.y,realpose_map.theta);
						//task_status = ROBOT_TASK_STS_COMPLETE;
//////////////////////////机器人最终定位完成
						return;
					}
				}////else/////是最终的结束段
			}////else if(i_step == 5)////////////////启动步骤5
			else if(i_step == 6)////////////////启动执行动作指令
			{
				if(move_flag == 1)
				{
				send_lift_msg(1);lift_up_finished = false;lift_down_finished = false;
				}
				else if(move_flag == 2)
				{
				send_lift_msg(0);lift_up_finished = false;lift_down_finished = false;
				}
				else if(move_flag == 0)
				{
				lift_down_finished = true;lift_down_finished = true;
				}

				printf("执行步骤66666666666666666\n");
				i_step = 7;
				usleep(500000);
				return;
			}
			else if(i_step == 7)////////////////等待动作指令执行完成
			{
				
				if(lift_up_finished||lift_down_finished)
				{
					printf("执行步骤77777777777777777\n");
					i_step = 0;////从当前段的旋转阶段开始运动
					runstep = 0;
					task_status = ROBOT_TASK_STS_COMPLETE;
				}
			}
		}////if(CURBEZIER.PTP_OR_CURVE == TRAJ_ISCURVE)
		
	}
}

void curve_run::nexttraj()/////切换到下一段轨迹
{
	printf("执行轨迹切换到辖一段轨迹程序\n");

	if(running_data.T_ENDPID < CURBEZIER.tarjp.total_u_num) return;/////如果当前轨迹并没有运行到最后的轨迹段，则不切换轨迹段
	TRAJ_RUNNING_STEP = TRAJ_RUN_STEPEVEN;/////取消减速段标识
	running_data.traj_index +=1;/////取下一条轨迹
	memset(&CURBEZIER, 0, sizeof(T_TRAJ_));	
	CURBEZIER = myspace_traj[running_data.traj_index];
	//////////////////////////////////////轨迹切换后，将目标点的id更新为新轨迹的终点id
	running_data.T_INIPRDY = false;////临时轨迹起始点READY
	running_data.T_ENDPRDY = false;////临时轨迹终止点READY
	running_data.T_NEXTPRDY = false;////临时轨迹下一点READY
	running_data.T_INIPID = 0;////临时轨迹起始点索引0
	running_data.T_ENDPID = 0;////临时轨迹终止点索引0
	running_data.T_NEXTPID = 0;////临时轨迹下一点索引0
	loadtrajpar();


	printf("切换下一段路径======================\n");
}

bool curve_run::jh_vel_test(double &line_vel,double &angle_vel)
{

	if(fabs(line_vel)>(nav_vel_line*vel_ratio))
	{
	if (line_vel>0) line_vel = nav_vel_line*vel_ratio;
	else line_vel = -nav_vel_line*vel_ratio;
	}
	if(fabs(angle_vel)>(vel_angle_recal*vel_ratio))
	{
	if (angle_vel>0) angle_vel = vel_angle_recal*vel_ratio;
	else angle_vel = -vel_angle_recal*vel_ratio;
	}
	return true;
}

void curve_run::exchangetraj()/////交换轨迹参数
{
		//////如果当前轨迹的终点索引等于total_u_num，说明当前已经取到最后轨迹点，则不再取下一点
		if(running_data.T_ENDPID > CURBEZIER.tarjp.total_u_num) 
		{
		return;
		}
		double total_len;
		
		if(running_data.T_NEXTPRDY == true&&running_data.T_ENDPID <= CURBEZIER.tarjp.total_u_num)
		{
		running_data.T_INIPRDY = false;
		running_data.T_ENDPRDY = false;
		running_data.T_INIP = running_data.T_ENDP;
		running_data.T_ENDP = running_data.T_NEXTP;
		running_data.T_INIPID = running_data.T_ENDPID;
		running_data.T_ENDPID = running_data.T_NEXTPID;
		running_data.T_INIPRDY = true;
		running_data.T_ENDPRDY = true;
		running_data.T_NEXTPRDY = false;
		////////////更新方向矢量
		
		if(running_data.T_INIPID < running_data.T_ENDPID)//////避免方向矢量更新为0,导致机器人停止的情况realpose_map
		dvect_between(running_data.T_ENDP,running_data.T_INIP,Traj.PTP_TARGET_DIR,total_len);
		}
		
		if(running_data.T_ENDPID >= CURBEZIER.tarjp.total_u_num&&running_data.T_INIPID < CURBEZIER.tarjp.total_u_num)
		dvect_between(CURBEZIER.tarjp.CURVE_TARGET_POSE,CURBEZIER.tarjp.ED_LKAHP,Traj.PTP_TARGET_DIR,total_len);
		
		loadtrajpar();

}




Position curve_run::cal_deltap_(char type,float dt,int &cur_t,int t_acc,int t_dec,int t_even,double l_even,double l_acc,double l_dec,double l_total,char motion_type,double &vrf,double &wrf,int &needsts)////由路径规划参数计算deltapos，vr和wr用于机器人的驱动
{
	Position targetp;
	Position deltap;
	Position chuizup;///垂足点
	double nor_t;
	double nor_s;
	double temp_len;
	int temp_jj;
	double targetangle_ = targetangle;
	double target_len;
	double target_theta_len;
	double delta_len;
	double temp_vel;
	float vel_total = CURBEZIER.vel_line;
	bool lr,ll,vr,vl;
	lr=ll=vr=vl=false;
	float delta_time = dt;
	if(delta_time<=10) delta_time = 20;

	float vel_total_ang = vel_angle_recal*0.5;//CURBEZIER.vel_ang;
	double angle_avoid;////避障触发时，计算得到的角速度
	int dirl = 1;
	int dirl2 = 1;
	float rate_apply = 1.5;
	double delta_angle_max = ANGLE_ADD_MAX*M_PI;
	double direct_inverse = 0.0;
	double x2car,y2car,theta,ctheta,stheta;
	double delta_p_angle;////当前的角度偏差
	double t_dec_temp;
  float delta_angle_running;
	int traj_type_cur = CURBEZIER.traj_type;
	int traj_type_endding = CURBEZIER.trisend;
	if(vel_total_ang<=min_angle_vel||vel_total_ang>vel_angle_recal)
	vel_total_ang = vel_angle_recal*0.5;

	if(vel_total<min_line_vel)
	vel_total = min_line_vel;
	else if(vel_total>nav_vel_line)
	vel_total = nav_vel_line;
	
	vel_ratio = vel_total/nav_vel_line;
	if(vel_ratio>1.0) vel_ratio = 1.0;

	if(traj_type_cur==TRAJ_START||traj_type_cur==TRAJ_MIDDLE)/////如果是起始或者过度，那么就不需要加上降速长度
	{dis_line_dec = 0.5*(vel_total*vel_total-min_line_vel*min_line_vel)/nav_dec_line;
	t_dec_temp = (vel_total-min_line_vel)/nav_dec_line*1000;
	}
	else
	{
	if(traj_type_endding != TRAJ_ISENDDING)
	dis_line_dec = 0.5*vel_total*vel_total/nav_dec_line+nav_slow_dis*vel_total;
	else
	dis_line_dec = 0.5*vel_total*vel_total/nav_dec_line+nav_slow_dis;
	t_dec_temp = (vel_total-min_line_vel)/nav_dec_line*1000;
	}

	if(CURBEZIER.isinverse == true)
	direct_inverse = 1.0;
	dis_between(CURBEZIER.tarjp.CURVE_TARGET_POSE,realpose_map,target_len);
	target_theta_len = (targetangle_-realpose_map.theta);
	running_time += delta_time;
	needsts = TRAJ_NEED_NEXTP_N;
	//delta_len = vel_total;
	
	delta_len = 1.0;
	if(CURBEZIER.PTP_OR_CURVE ==TRAJ_ISFREE)
	delta_len = 0.3;
	if (delta_len <0.5)delta_len =0.5;
	if (delta_len >1.0)delta_len =1.0;
	{
		if(running_time<=t_acc&&TRAJ_RUNNING_STEP != TRAJ_RUN_STEPDEC)/////////加速阶段
		{nor_t = (double(running_time))*0.5/(double(t_acc));
		nor_s = msine(nor_t);
		TRAJ_RUNNING_STEP = TRAJ_RUN_STEPACC;
		if(type == TRAJ_LINE)
		{//////如果是直行运动，则需要重新计算目标点及其相对于机器人坐标系的位置
		if(CURBEZIER.PTP_OR_CURVE != TRAJ_ISCURVE &&(CURBEZIER.PTP_OR_CURVE != TRAJ_ISFREE))////如果当前轨迹是直线
		chuizup = cal_CZ(CURBEZIER.tarjp.CURVE_INITIAL_POSE,CURBEZIER.tarjp.CURVE_TARGET_POSE,realpose_map,dirl);
		else
		chuizup = cal_CZ(running_data.T_INIP,running_data.T_ENDP,realpose_map,dirl);
	
		targetp.x=chuizup.x+Traj.PTP_TARGET_DIR.x*delta_len;
		targetp.y=chuizup.y+Traj.PTP_TARGET_DIR.y*delta_len;
		targetp.theta=atan2(targetp.y-realpose_map.y,targetp.x-realpose_map.x)-M_PI*(direct_inverse);//20191009更改-M_PI*(0.5+direct_inverse);
    if(targetp.theta>M_PI)	targetp.theta-=2*M_PI;
		else if(targetp.theta<-M_PI)	targetp.theta+=2*M_PI;

		//////////////////////////////将该坐标值转换到机器人局部坐标系下
		deltap = (targetp - realpose_map);
		theta = realpose_map.theta-M_PI*(direct_inverse);;//将偏差转换到第一四相限+M_PI*(0.5-direct_inverse);
		ctheta = cos(theta);stheta = sin(theta);
		x2car = ctheta*deltap.x + stheta*deltap.y;
		y2car = -stheta*deltap.x + ctheta*deltap.y;
		delta_p_angle = atan2(y2car,x2car);

		delta_angle_running = delta_p_angle;
		////////////////////////////////
		
		if((traj_type_cur==TRAJ_END||traj_type_cur==TRAJ_MIDDLE)&&(CURBEZIER.PTP_OR_CURVE != TRAJ_ISPTP))
		vrf = vel_total*2*nor_s+vel_start*(1-2*nor_s);
		else
		vrf = vel_total*2*nor_s;//+min_line_vel*(1-2*nor_s);//auto_run_lvel
		//printf("执行直线pppppppptppppppppppppppppp过程%f，dis_line_dec=%f,target_len=%f\n",vrf,dis_line_dec,target_len);
		cmd_line_vel = vrf;

		if(delta_angle_running>delta_angle_max) delta_angle_running = delta_angle_max;
		else if(delta_angle_running<-delta_angle_max) delta_angle_running = -delta_angle_max;
		
		wrf = delta_angle_running*LYP_KPHY1;/////直接取当前角度值
		wrf =wrf/(0.001*delta_time);
		}
		if(type == TRAJ_ROTATE)/////如果是旋转运动
		{
	  if(fabs(target_theta_len)<(M_PI*THETA_TORLERANCE)||fabs(target_theta_len+2*M_PI)<(M_PI*THETA_TORLERANCE)||fabs(target_theta_len-2*M_PI)<(M_PI*THETA_TORLERANCE))  
		{running_time=0;
		
		TRAJ_RUNNING_STEP = TRAJ_RUN_STEPDEC;
		printf("旋转加速阶段进入减速步骤//--------------------------\n");
		}////当距离目标点10度时开始减速
		vrf = 0.0;
		cmd_line_vel = vrf;
		wrf = vel_total_ang*2*nor_s;
		cmd_angle_vel = wrf;
		}
		else
		{
			if(target_len<dis_line_dec)//&&(traj_type_cur!=TRAJ_START&&traj_type_cur!=TRAJ_MIDDLE)) 
			{running_time = 0;
			TRAJ_RUNNING_STEP = TRAJ_RUN_STEPDEC;
			printf("111距离目标点足够近target_len==%f，直线加速阶段进入减速%f步骤////////////////////////\n",target_len,dis_line_dec);
			}
      if(dis_among(realpose_map,running_data.T_ENDP,pass_allow_dis)||dirl<0)
			needsts = TRAJ_NEED_NEXTP_Y;
		}
		temp_jj = 111;
		}
		else if(TRAJ_RUNNING_STEP != TRAJ_RUN_STEPDEC)//////////匀速阶段
		{
		nor_t = 0.5;
		if(type == TRAJ_LINE)
		{//////如果是直线运动，则需要重新计算目标点及其相对于机器人坐标系的位置
		if(CURBEZIER.PTP_OR_CURVE != TRAJ_ISCURVE &&(CURBEZIER.PTP_OR_CURVE != TRAJ_ISFREE))////如果当前轨迹是直线
		chuizup = cal_CZ(CURBEZIER.tarjp.CURVE_INITIAL_POSE,CURBEZIER.tarjp.CURVE_TARGET_POSE,realpose_map,dirl);
    else
		chuizup = cal_CZ(running_data.T_INIP,running_data.T_ENDP,realpose_map,dirl);

		targetp.x=chuizup.x+Traj.PTP_TARGET_DIR.x*delta_len;
		targetp.y=chuizup.y+Traj.PTP_TARGET_DIR.y*delta_len;
		targetp.theta=atan2(targetp.y-realpose_map.y,targetp.x-realpose_map.x)-M_PI*(direct_inverse);//20191009更改-M_PI*(0.5+direct_inverse);
    if(targetp.theta>M_PI)	targetp.theta-=2*M_PI;
		else if(targetp.theta<-M_PI)	targetp.theta+=2*M_PI;
//////////////////////////////将该坐标值转换到机器人局部坐标系下
		deltap = (targetp - realpose_map);
		theta = realpose_map.theta-M_PI*(direct_inverse);//将偏差转换到第一四相限+M_PI*(0.5-direct_inverse);
		ctheta = cos(theta);stheta = sin(theta);
		x2car = ctheta*deltap.x + stheta*deltap.y;
		y2car = -stheta*deltap.x + ctheta*deltap.y;
		delta_p_angle = atan2(y2car,x2car);
		delta_angle_running = delta_p_angle;
		////////////////////////////////
		//if(running_time%5 == 0)
		//printf("执行直线cccccccccccccccc过程%f，dis_line_dec=%f,target_len=%f,TRAJ_RUNNING_STEP=%d\n",vrf,dis_line_dec,target_len,TRAJ_RUNNING_STEP);
		vrf = vel_total;
		cmd_line_vel = vrf;
	
		if(delta_angle_running>delta_angle_max) delta_angle_running = delta_angle_max;
		else if(delta_angle_running<-delta_angle_max) delta_angle_running = -delta_angle_max;
		wrf = delta_angle_running*LYP_KPHY1;/////直接取当前角度值
		//wrf =wrf/(0.001*delta_time)*vrf;
		wrf =wrf/(0.001*delta_time);
		//wrf =wrf/(0.001*dt);
		}
		if(type == TRAJ_ROTATE)/////如果是旋转运动，当接近10度时，启动减速
		{
		if(fabs(target_theta_len)<(M_PI*THETA_TORLERANCE)||fabs(target_theta_len+2*M_PI)<(M_PI*THETA_TORLERANCE)||fabs(target_theta_len-2*M_PI)<(M_PI*THETA_TORLERANCE))
		{running_time=0;
		TRAJ_RUNNING_STEP = TRAJ_RUN_STEPDEC;
		printf("旋转匀速阶段进入减速步骤//+++++++++++++++++++++++++++++++\n");
		}////当距离目标点10度时开始减速
		vrf = 0.0;
		cmd_line_vel = vrf;
		wrf = vel_total_ang;
		cmd_angle_vel = wrf;
		}
		else
		{

			if(target_len<dis_line_dec)//&&(traj_type_cur!=TRAJ_START&&traj_type_cur!=TRAJ_MIDDLE)) 
			{running_time = 0;
			TRAJ_RUNNING_STEP = TRAJ_RUN_STEPDEC;
			printf("222距离目标点足够近target_len==%f，直线匀速阶段进入减速步骤////////////////////////\n",target_len);}

	
			if(dis_among(realpose_map,running_data.T_ENDP,pass_allow_dis)||dirl<0)
				needsts = TRAJ_NEED_NEXTP_Y;

		}
		temp_jj = 222;

		}
		else
		{

		nor_t = (double(running_time))/(double(t_dec_temp*2))+0.5;//+(1-last_running_adds_time);
		nor_s = msine(nor_t);
		if(target_len<nav_slow_dis||(type == TRAJ_ROTATE))
		//delta_len = 0.5;
		if(TRAJ_RUNNING_STEP == TRAJ_RUN_STEPDEC)
		delta_len = 0.5;


		if(type == TRAJ_LINE)
		{//////如果是直线运动，则需要重新计算目标点及其相对于机器人坐标系的位置
		if(CURBEZIER.PTP_OR_CURVE != TRAJ_ISCURVE &&(CURBEZIER.PTP_OR_CURVE != TRAJ_ISFREE))////如果当前轨迹是直线
		chuizup = cal_CZ(CURBEZIER.tarjp.CURVE_INITIAL_POSE,CURBEZIER.tarjp.CURVE_TARGET_POSE,realpose_map,dirl);
		else
		chuizup = cal_CZ(running_data.T_INIP,running_data.T_ENDP,realpose_map,dirl);
		targetp.x=chuizup.x+Traj.PTP_TARGET_DIR.x*delta_len;
		targetp.y=chuizup.y+Traj.PTP_TARGET_DIR.y*delta_len;
		targetp.theta=atan2(targetp.y-realpose_map.y,targetp.x-realpose_map.x)-M_PI*(direct_inverse);//20191009更改-M_PI*(0.5+direct_inverse);
    if(targetp.theta>M_PI)	targetp.theta-=2*M_PI;
		else if(targetp.theta<-M_PI)	targetp.theta+=2*M_PI;
		
		//////////////////////////////将该坐标值转换到机器人局部坐标系下
		deltap = (targetp - realpose_map);
		theta = realpose_map.theta-M_PI*(direct_inverse);;//将偏差转换到第一四相限+M_PI*(0.5-direct_inverse);
		ctheta = cos(theta);stheta = sin(theta);
		x2car = ctheta*deltap.x + stheta*deltap.y;
		y2car = -stheta*deltap.x + ctheta*deltap.y;
		delta_p_angle = atan2(y2car,x2car);
		delta_angle_running = delta_p_angle;
		////////////////////////////////

		temp_vel = target_len/(0.001*delta_time)*0.1;
		if(temp_vel>min_line_vel) temp_vel = min_line_vel;
		vrf = cmd_line_vel*2*(1-nor_s)+temp_vel*(1-2*(1-nor_s));

		if(delta_angle_running>delta_angle_max) delta_angle_running = delta_angle_max;
		else if(delta_angle_running<-delta_angle_max) delta_angle_running = -delta_angle_max;
		//if(slow_dis_step != true)
		wrf = delta_angle_running*LYP_KPHY1;/////直接取当前角度值
    wrf =wrf/(0.001*delta_time);
		//wrf =wrf/(0.001*delta_time)*vrf;//*LYP_KP_DEC;
		}
		if(dirl<0&&CURBEZIER.PTP_OR_CURVE != TRAJ_ISCURVE &&(CURBEZIER.PTP_OR_CURVE != TRAJ_ISFREE))////如果是直线运动，则超过终点反向驱动，否则不反向
		{//////若当前点已经超过目标点，则反向驱动
		vrf = -vrf*0.5;
		wrf = -wrf;/////
		}
		else if(dirl<0&&
		running_data.T_NEXTPID>=CURBEZIER.tarjp.total_u_num)////如果是贝塞尔运动，则超过终点反向驱动，否则不反向
		{//////若当前点已经超过目标点，则反向驱动
		vrf = -vrf*0.5;
		wrf = -wrf;/////
		target_reached_count++;
		}
		if(type == TRAJ_ROTATE)
		{
		vrf = 0.0;
		wrf = cmd_angle_vel*2*(1-nor_s)+min_angle_vel*(1-2*(1-nor_s));
		}
		else
		{
			if(CURBEZIER.PTP_OR_CURVE == TRAJ_ISCURVE ||(CURBEZIER.PTP_OR_CURVE == TRAJ_ISFREE))////如果当前轨迹是曲线，判断临时终点是否到达
			if(dis_among(realpose_map,running_data.T_ENDP,pass_allow_dis)||dirl<0)
			//if(running_data.T_ENDPID < CURBEZIER.tarjp.total_u_num) //////当前点u还没有到达最终点u
			needsts = TRAJ_NEED_NEXTP_Y;
		}
		///////在减速阶段，修正targetp为目标点，用于判断目标接近情况，当x方向满足条件即认为是目标达到
		targetp = CURBEZIER.tarjp.CURVE_TARGET_POSE;
		//////////////////////////////将该坐标值转换到机器人局部坐标系下
		deltap = (targetp - realpose_map);
		theta = realpose_map.theta+M_PI*(-direct_inverse);;//将偏差转换到第一四相限+M_PI*(0.5-direct_inverse);
		ctheta = cos(theta);stheta = sin(theta);
		x2car = ctheta*deltap.x + stheta*deltap.y;
		y2car = -stheta*deltap.x + ctheta*deltap.y;
		//wrf = targetp.theta-ANGLE_MEAN_RECORD;/////直接取滤波后的当前角度值
		temp_jj = 333;
		//TRAJ_RUNNING_STEP = TRAJ_RUN_STEPDEC;
		}
		deltap = (targetp - realpose_map);
		deltap.x = x2car;
		deltap.y = y2car;
		if(wrf>M_PI) wrf-=2*M_PI; else if(wrf<-M_PI) wrf += 2*M_PI;
		//deltap.theta = wrf;
		deltap.theta = delta_p_angle;
		//vrf =vrf/(0.001*dt);
		//wrf =wrf/(0.001*dt);
		//printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
		//printf("当前targetp.x=%f,targetp.y = %f,targetp.theta=%f,delta_target= %f,nor_s=%f\n",targetp.x,targetp.y,targetp.theta,target_len,nor_s);
		//printf("当前cur_t=%d,jiduan=%d,temp_len = %f,deltax=%f,deltay=%f,deltatheta=%f，vrf=%f,wrf=%f\n",running_time,temp_jj,temp_len,deltap.x,deltap.y,deltap.theta,vrf,wrf);
		//printf("当前realpose_map_x = %f,realpose_map_y = %f,realpose_map_z = %f,targetangle=%f\n",realpose_map.x,realpose_map.y,realpose_map.theta,targetangle);
		//printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");

		////////////////////////////////////////////////////////
		if(CURBEZIER.isinverse == true && type != TRAJ_ROTATE)/////对于旋转运动)
		{
			vrf = -vrf;
			wrf = wrf;/////
		}
		if(type == TRAJ_LINE)
		wrf = wrf*nav_vel_angle_p;/////将计算得到的角速度乘上比例因子
		//printf("22222nor_s=%f,nor_t=%f,t_acc=%d,vrf=%f,wrf=%f\n",nor_s,nor_t,t_acc,vrf,wrf);
		return deltap;
	}
}
float curve_run::msine(float nor_time)
{
	float	nor_s;
	float	nor_t;
	
	nor_t=fabs(nor_time);
	if (nor_t>1) return 1.0;/////归一化时间计算出错

	//nor_s=nor_t-sin(2*M_PI*nor_t)/(2*M_PI);
	nor_s = nor_t;
	return nor_s;

}
bool curve_run::JH_LEN2T(float len,char type,int &t_acc,int &t_dec,int &t_even,double &l_even,double &l_acc,double &l_dec,double &l_total,char &motion_type)//由运行路程计算该轨迹的运动参数
{
	if (type<0||type>1) return false;////轨迹类型出错0----直线，1-----旋转
	double max_speed;
	double acc;
	double dec;
	double tacc,tdec;
	bool b_even_exist=false;
	double l_acdc;
	double l_ac_js;
	//len = CURBEZIER.tarjp.length_read*0.001;
	float vel_total = CURBEZIER.vel_line;
	float vel_total_ang = CURBEZIER.vel_ang;
	
	if(vel_total_ang<=min_angle_vel||vel_total_ang>vel_angle_recal)
	//vel_total_ang = nav_vel_angle*vel_ratio;
	vel_total_ang =vel_angle_recal*0.5;
	if(vel_total<=min_line_vel)
	vel_total = min_line_vel;
	else if(vel_total>nav_vel_line)
	vel_total = nav_vel_line;

	if(type == 0)
	{	//max_speed=nav_vel_line*vel_ratio;/////取线速度
		max_speed=vel_total;
		acc = nav_acc_line;
		dec = nav_dec_line;
		printf("当前路径长度=%f,线速度=%f,加速度=%f，减速度=%f\n",len,max_speed,acc,dec);
	}
	else if(type == 1)
	{	//max_speed=nav_vel_angle*vel_ratio;/////取角速度
		max_speed=vel_total_ang;
		acc = nav_acc_angle;
		dec = nav_dec_angle;
		printf("当前角速度=%f  ，加速度=%f，减速度=%f\n",max_speed,acc,dec);
	}
	tacc = max_speed/acc;
	tdec = max_speed/dec;
	//l_ac_js = 0.5*tacc*tacc*acc+nav_slow_dis;
	l_acdc = 0.5*tacc*tacc*acc+0.5*tdec*tdec*dec;
	l_ac_js = l_acdc+nav_slow_dis;
	//printf("计算000得到tacc=%f,tdec=%f,l_ac_js=%f,l_acdc=%f,ltotal=%f》》》\n",tacc,tdec,l_ac_js,l_acdc,len,motion_type);
	if(fabs(l_ac_js)<=fabs(len)&&type == 0)////////存在降速阶段并且是直线运动
	{
		//////修正DEC
		//dec = max_speed*max_speed/nav_slow_dis*0.5;
		motion_type=MOTION_COMPLETE;
		tacc = (max_speed/acc*1000);
		tdec = (max_speed/dec*1000);
		l_acc = 0.5*acc*tacc*tacc*0.001*0.001;
		l_dec = 0.5*dec*tdec*tdec*0.001*0.001;
		//l_dec = nav_slow_dis;
		t_acc = floor(tacc);
		t_dec = floor(tdec);
		t_even = floor((len-l_acc-l_dec)/max_speed*1000);
		l_even =len -l_acc -l_dec;
		l_total=len;
		printf("计算规划111111得到t_even=%d,tdec=%f,t_acc=%d,t_dec=%d,ldec=%f,lacc=%f,ltotal=%f,motiontype=%d》》》\n",t_even,tdec,t_acc,t_dec,l_dec,l_acc,l_total,motion_type);
		return true;
	}
	else///////不存在降速阶段或者是旋转运动
	{
		if (fabs(l_acdc)>=fabs(len))///判断是否存在匀速时间
		{///不存在匀速时间
			motion_type=MOTION_ACCDEC;
			tacc = sqrt(len/(0.5*acc+0.5*acc*acc/dec))*1000;
			tdec = acc*tacc/dec;
			t_acc=floor(tacc);
			t_dec=floor(acc*t_acc/dec);
			t_even=0;
			l_even=0;
			l_acc=dec/(acc+dec)*len;
			l_dec=acc/(acc+dec)*len;
			l_dec = len*0.5;
			l_total=len;
			printf("计算规划222得到t_even=%d,tdec=%f,t_acc=%d,t_dec=%d,ldec=%f,ltotal=%f,motiontype=%d》》》\n",t_even,tdec,t_acc,t_dec,l_dec,l_total,motion_type);
			return true;
		}
		else/////存在匀速时间
		{
			motion_type=MOTION_COMPLETE;
			tacc=(max_speed/acc*1000);
			tdec=(max_speed/dec*1000);

			l_acc=0.5*acc*tacc*tacc*0.001*0.001;
			l_dec=0.5*dec*tdec*tdec*0.001*0.001;

			t_acc = floor(tacc);
			t_dec = floor(tdec);
			t_even=floor((len-l_acc-l_dec)/max_speed*1000);

			l_acc=l_acc;
			l_dec=l_dec;
			l_even=len-l_acc-l_dec;
			l_total=len;
			printf("计算333得到t_even=%d,tdec=%f,t_acc=%d,t_dec=%d,ldec=%f,l_even=%f,ltotal=%f,motiontype=%d》》》\n",t_even,tdec,t_acc,t_dec,l_dec,l_even,l_total,motion_type);
			return true;
		}
	}
	


	return false;
}