#include <cmath>
#include <memory>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include "agv.h"
#include "pallet_recgnize.h"
//FILE *fp_scan_position;

Pallet_recgnize::Pallet_recgnize(MsgQueue *queue, int radar_chan)
{
    this->queue_a = queue;
    radar_c = radar_chan;
    queue_a->add_listener(radar_c, this);
	pthread_mutex_init(&listen_pose_mutex, nullptr);
	//fp_scan_position = fopen("radar_scan_pose.txt","w");
    para_initial();
}
Pallet_recgnize::~Pallet_recgnize()
{
	pthread_mutex_destroy(&listen_pose_mutex);
	queue_a->remove_listener(radar_c, this);
		
}
float Pallet_recgnize::cal_disp(double x1,double y1)
{
	float lx = x1;
	float ly =y1;
	float len_temp =sqrt(lx*lx+ly*ly);
	return len_temp;
}
int Pallet_recgnize::radar_data_dispose()
{
	PointCloudData cornerPointsSharp(0,100);
	std::vector<int> scanStartInd(1, 0);
 	std::vector<int> scanEndInd(1, 0);
	int cloudSize = last_radar_data.points.size();
	float len1,len2,len3,len4,len5,len6,len;
	float dis_tot;
	for (int i = 5; i < cloudSize - 5; i++) {

	len1 = cal_disp(last_radar_data.points[i - 3](0),last_radar_data.points[i - 3](1));
	len2 = cal_disp(last_radar_data.points[i - 2](0),last_radar_data.points[i - 2](1));
	len3 = cal_disp(last_radar_data.points[i - 1](0),last_radar_data.points[i - 1](1));
	len4 = cal_disp(last_radar_data.points[i + 1](0),last_radar_data.points[i + 1](1));
	len5 = cal_disp(last_radar_data.points[i + 2](0),last_radar_data.points[i + 2](1));
	len6 = cal_disp(last_radar_data.points[i + 3](0),last_radar_data.points[i + 3](1));
	len =  cal_disp(last_radar_data.points[i ](0),last_radar_data.points[i ](1));
	dis_tot = len1+len2+len3+len4+len5+len6 - 6*len;
	cloudCurvature[i] = dis_tot *dis_tot ;
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;
  }
	int serch_cnt_record = ((end_ang - ini_ang)*0.5 -search_angle)*2 *0.5;///（总范围 - 搜索范围）*0.5
	if(serch_cnt_record<15) serch_cnt_record =15;////避免搜索起始索引太小
	scanStartInd[0] = serch_cnt_record;
	scanEndInd[0] = cloudSize - serch_cnt_record;
	cornerPointsSharp.points.clear();
	cornerPointsSharp.intensities.clear();
	//angle_min = -fabs(end_ang - ini_ang)*ang_step / 2 ;
	///////////将整个激光区域分为6部分，取每部分曲率前4大的点存储
	for (int jj = serch_cnt_record; jj < cloudSize - serch_cnt_record; jj++) {
      if (cloudNeighborPicked[jj] == 0 &&
            cloudCurvature[jj] > 0.5) 
     {
		float x1 = last_radar_data.points[jj](0);
		float y1 = last_radar_data.points[jj](1);
		float th1 = atan2(y1,x1)/M_PI*180;
		float dis1 = sqrt(x1*x1+y1*y1);
		if((fabs(th1)<(search_angle/2))&&(dis1>0.1)&&(dis1<3.0))
		{
		 cornerPointsSharp.add_point(last_radar_data.points[jj](0),last_radar_data.points[jj](1),jj);
		printf("cur x=%f,y=%f,th1=%f,dis1=%f,ind=%d\n",x1,y1,th1,dis1,jj);
		}
         cloudNeighborPicked[jj] = 1;
        
       }

    }
////////////////////计算特征点中距离机器人当前位置最近的点作为托盘的一个腿点
	Position foot1_position;
	Position foot2_position;
	int foot1_ind,foot2_ind;
	int cornerPointsSharpNum = cornerPointsSharp.points.size();
	 printf("cornerPointsSharpNum =%d,laserpoint_num=%d\n",cornerPointsSharpNum,last_radar_data.points.size());
	float pointSqDis, minPointSqDis2;
	int closestPointInd = 0, minPointInd2 = -1;
	minPointSqDis2 = (cornerPointsSharp.points[0](0) ) * (cornerPointsSharp.points[0](0) ) + 
					(cornerPointsSharp.points[0](1) ) * (cornerPointsSharp.points[0](1) ) ;

	for (int j = 1; j < cornerPointsSharpNum; j++)
	{
		
		pointSqDis = (cornerPointsSharp.points[j](0) ) * 
					(cornerPointsSharp.points[j](0) ) + 
					(cornerPointsSharp.points[j](1) ) * 
					(cornerPointsSharp.points[j](1) ) ;
		if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      closestPointInd = j;
			//printf("closestPointInd =%d\n",j);
                    }
	}
	Position pointSel;
	foot1_position.x = cornerPointsSharp.points[closestPointInd](0);
	foot1_position.y = cornerPointsSharp.points[closestPointInd](1);
	foot1_ind = int(cornerPointsSharp.intensities[closestPointInd]);
	pointSel.x = foot1_position.x;
	pointSel.y = foot1_position.y;
	float xx,yy;
	float minidis;
	float thi_foot;
	float thi_mini;
	//////////////////////////从foot1点开始搜索，第二个腿点，
	xx = cornerPointsSharp.points[closestPointInd+1](0) - pointSel.x;
	yy = cornerPointsSharp.points[closestPointInd+1](1) - pointSel.y;
	minPointInd2 = closestPointInd+1;
	//minPointSqDis2 = fabs(pallet_width - sqrt(xx*xx+yy*yy));

	thi_foot = atan2(yy,xx);
	minPointSqDis2 = fabs(fabs(thi_foot)-M_PI/2);
	
	for (int j = closestPointInd + 2; j < cornerPointsSharpNum; j++) 
	{                  
		pointSqDis = (cornerPointsSharp.points[j](0) - pointSel.x) * 
					(cornerPointsSharp.points[j](0) - pointSel.x) + 
					(cornerPointsSharp.points[j](1) - pointSel.y) * 
					(cornerPointsSharp.points[j](1) - pointSel.y) ;

		minidis = fabs(pallet_width - sqrt(pointSqDis));
		thi_foot = atan2(cornerPointsSharp.points[j](1) - pointSel.y,cornerPointsSharp.points[j](0) - pointSel.x);
		thi_mini = fabs(fabs(thi_foot)-M_PI/2);
		//if ( minidis < minPointSqDis2&&(fabs(fabs(thi_foot)-M_PI/2)<0.4))
		if ( minidis < 0.1 && thi_mini<minPointSqDis2)
		{
			minPointSqDis2 = thi_mini;
			minPointInd2 = j;
			printf("111minidis =%f,ind=%d,thi_foot=%f\n",minidis ,j,thi_foot);
		}

	}

	for (int j = closestPointInd - 1; j >= 0; j--) 
	{
		pointSqDis = (cornerPointsSharp.points[j](0) - pointSel.x) * 
					(cornerPointsSharp.points[j](0) - pointSel.x) + 
					(cornerPointsSharp.points[j](1) - pointSel.y) * 
					(cornerPointsSharp.points[j](1) - pointSel.y);
		minidis = fabs(pallet_width - sqrt(pointSqDis));
		thi_foot = atan2(cornerPointsSharp.points[j](1) - pointSel.y,cornerPointsSharp.points[j](0) - pointSel.x);
		thi_mini = fabs(fabs(thi_foot)-M_PI/2);

		//if (minidis < minPointSqDis2&&(fabs(fabs(thi_foot)-M_PI/2)<0.4)) 
		if ( minidis < 0.1 && thi_mini<minPointSqDis2)
		{
			minPointSqDis2 = thi_mini;
			minPointInd2 = j;
			printf("222minidis =%f,ind=%d,thi_foot=%f\n",minidis ,j,thi_foot);
		}
	}
	foot2_position.x = cornerPointsSharp.points[minPointInd2](0);
	foot2_position.y = cornerPointsSharp.points[minPointInd2](1);
	foot2_ind = int(cornerPointsSharp.intensities[minPointInd2]);
////////////////////////////////////////计算两个foot点之间连线的角度值，如果角度太偏则认为没有识别到托盘
	
	printf("foot1 pos x=%f,y=%f,ind=%d\n",foot1_position.x,foot1_position.y,foot1_ind);
	printf("foot2 pos x=%f,y=%f,ind=%d\n",foot2_position.x,foot2_position.y,foot2_ind);
	thi_foot = atan2((foot2_position.y-foot1_position.y),(foot2_position.x-foot1_position.x));
	if(fabs(fabs(thi_foot)-M_PI/2)>0.7||(foot1_ind==foot2_ind)) 
	{
		printf("!!!!!!!!!!!! theta=%f is too big or foot1_ind ==foot2_ind\n",thi_foot);
		return -1;
	}
	if(fabs(minPointSqDis2 )>0.5*pallet_width)	
	{
		printf("!!!!!!!!!!!!delta len=%f is too big\n",minPointSqDis2 );
		return -1;
	}
////////////////////////////////////////分别取特征1点，特征2点附近的点簇均值，作为托盘腿部的坐标
	mean_foot1 = foot1_position;
	Position total_foot1 = foot1_position;
	mean_foot2 = foot2_position;
	Position total_foot2 = foot2_position;
	int foots_cnt1 = 1;
	int foots_cnt2 = 1;
	for (int j = serch_cnt_record; j < cloudSize - serch_cnt_record; j++) 
	{

		//float potinSqDis1 = (last_radar_data.points[j](0) - mean_foot1.x) * 
		//			(last_radar_data.points[j](0) - mean_foot1.x) + 
		//			(last_radar_data.points[j](1) - mean_foot1.y) * 
		//			(last_radar_data.points[j](1) - mean_foot1.y) ;
		//float potinSqDis2 = (last_radar_data.points[j](0) - mean_foot2.x) * 
		//			(last_radar_data.points[j](0) - mean_foot2.x) + 
		//			(last_radar_data.points[j](1) - mean_foot2.y) * 
		//			(last_radar_data.points[j](1) - mean_foot2.y) ;
		if(fabs(last_radar_data.points[j](0) - mean_foot1.x)<0.1&&fabs(last_radar_data.points[j](1) - mean_foot1.y)<0.03)
		{
			foots_cnt1++;
			total_foot1.x+=last_radar_data.points[j](0);
			total_foot1.y+=last_radar_data.points[j](1);
			mean_foot1.x = total_foot1.x/foots_cnt1;
			mean_foot1.y = total_foot1.y/foots_cnt1;
			//printf("foots_cnt1=%d x=%f,y=%f,mean_foot1.x=%f,mean_foot1.y=%f\n",foots_cnt1,last_radar_data.points[j](0),last_radar_data.points[j](1),mean_foot1.x,mean_foot1.y);
		}
		if(fabs(last_radar_data.points[j](0) - mean_foot2.x)<0.1&&fabs(last_radar_data.points[j](1) - mean_foot2.y)<0.03)
		{
			/* code */
			foots_cnt2++;
			total_foot2.x+=last_radar_data.points[j](0);
			total_foot2.y+=last_radar_data.points[j](1);
			mean_foot2.x = total_foot2.x/foots_cnt2;
			mean_foot2.y = total_foot2.y/foots_cnt2;
			//printf("foots_cnt2=%d x=%f,y=%f,mean_foot2.x=%f,mean_foot2.y=%f\n",foots_cnt2,last_radar_data.points[j](0),last_radar_data.points[j](1),mean_foot2.x,mean_foot2.y);
		}
	}
	printf("mean_foot1 pos x=%f,y=%f,total=%d\n",mean_foot1.x,mean_foot1.y,foots_cnt1);
	printf("mean_foot2 pos x=%f,y=%f,total=%d\n",mean_foot2.x,mean_foot2.y,foots_cnt2);
	///////////////////////////////////////////////////
	thi_foot = atan2((mean_foot2.y-mean_foot1.y),(mean_foot2.x-mean_foot1.x));
	if(fabs(fabs(thi_foot)-M_PI/2)>0.7) 
	{
		printf("22222222222!!!!! theta=%f is too big\n",thi_foot);
		return -1;
	}
	xx = mean_foot2.x-mean_foot1.x;
	yy = mean_foot2.y-mean_foot1.y;
	minidis = sqrt(xx*xx+yy*yy);
	printf("mean_foot2---mean_foot1 len=%f \n",minidis );
	if(fabs(minidis)<0.5*pallet_width)	
	{
		printf("22222222222!!!!!! len=%f is too small\n",minidis);
		return -1;
	}
	
	return 1;
}
bool Pallet_recgnize::para_initial()
{
	radar_position.x = 0.3;
	radar_position.y = 0;
	radar_position.theta = 0;
	ini_ang = 60;
	end_ang = 480;
	ang_step = 0.5;
}
void Pallet_recgnize::recieve_message(const int channel, char *buf, const int size)
{   
 //   pthread_mutex_lock(&listen_pose_mutex);
	int result;
     if (channel == CHANNEL_RADAR){//得到scanmatch
        PointCloudData data(0);
        data.from_char_array(buf, size);
       		//for(auto & point : data.points) {
			//point(0) += radar_position.x;
		//}
	//printf(" received radar data\n");
	laser_recv_cnt++;
	if(recognized_start == true)
	{	//fprintf(fp_scan_position,"\n scan pose\n");
		//for(int j=0;j<data.points.size();j++)
		//fprintf(fp_scan_position,"%f,%f\n",data.points[j](0),data.points[j](1));

		printf("radar data received\n");
        	last_radar_data = data;
		result = radar_data_dispose();
		if(result == -1)///////托盘腿部识别失败
		{
			recognize_cnt++;
			if(recognize_cnt >= max_recog_cnt) recognized_start = false;
		}
		else if(result == 1)/////托盘腿部识别成功
		{
			recognize_cnt = 0;
			pose_recognized = true;
			recognized_start = false;
		}
	}
	}
 //   pthread_mutex_unlock(&listen_pose_mutex);
}

int Pallet_recgnize::pallet_recognize(float len,float width,float angle_sech,int clc_cnt,Position & result_pose)////
{

	recognized_start = true;
	pose_recognized = false;
	recognize_cnt = 0;
	max_recog_cnt = clc_cnt;
	pallet_width = width;
	pallet_lenth = len;
	search_angle = angle_sech;
	printf("recv pallet_lenth=%f,pallet_width=%f,search_angle=%f,max_recog_cnt=%d\n",pallet_lenth,pallet_width,search_angle,max_recog_cnt);
	Position pallet2map;
	result_pose = pallet2map;
	while(recognized_start == true) usleep(10000);
	if(pose_recognized == false) return -1;
//////////////////////////////////根据两个foot的均值，重新计算托盘中心的位姿数据
	Position robotp1 = get_global_agv_instance()->expolate_current_position();
	Position mean_foot1_f = mean_foot1;
	Position mean_foot2_f = mean_foot2;
	Position pallet2agv=mean_foot1;
	pallet2agv = pallet2agv + mean_foot2;
	float xx,yy,thi_foot;
	//mean_foot1_f.y+=pallet_lenth;
	//mean_foot2_f.y+=pallet_lenth;
	xx = mean_foot1.x-mean_foot2.x;
	yy = mean_foot1.y-mean_foot2.y;
	float dlen=sqrt(xx*xx+yy*yy);
	float dirx,diry;
	if(mean_foot1.y<mean_foot2.y)
	{
		dirx = -(mean_foot1.y-mean_foot2.y)/dlen;
		diry = (mean_foot1.x-mean_foot2.x)/dlen;
		thi_foot = atan2(diry ,dirx );
	}
	else
	{
		/* code */
		dirx = -(mean_foot2.y-mean_foot1.y)/dlen;
		diry = (mean_foot2.x-mean_foot1.x)/dlen;
		thi_foot = atan2(diry ,dirx );
	}
	mean_foot1_f.x+=dirx*pallet_lenth;
	mean_foot1_f.y+=diry*pallet_lenth;
	//printf("mean_foot1_f x =%f,y=%f,dirx =%f,diry =%f\n",mean_foot1_f.x,mean_foot1_f.y,dirx ,diry);
	pallet2agv = pallet2agv + mean_foot1_f;
	mean_foot2_f.x+=dirx*pallet_lenth;
	mean_foot2_f.y+=diry*pallet_lenth;
	//printf("mean_foot1_f x =%f,y=%f,dirx =%f,diry =%f\n",mean_foot2_f.x,mean_foot2_f.y,dirx ,diry);
	pallet2agv = pallet2agv + mean_foot2_f;

	pallet2agv.x = pallet2agv.x/4-radar_position.x;
	pallet2agv.y = pallet2agv.y/4;
	pallet2agv.theta = thi_foot;
	printf("agv_pose pos x=%f,y=%f,theta=%f\n",robotp1.x,robotp1.y,robotp1.theta);
	printf("pallet reletive to agv pos x=%f,y=%f,theta=%f\n",pallet2agv.x,pallet2agv.y,pallet2agv.theta);
	/////////////////////////////////将识别出的托盘位置转换到，全局坐标系下
	pallet2map = robotp1*pallet2agv;
	result_pose = pallet2map;
	printf("result_pose pos x=%f,y=%f,theta=%f\n",result_pose.x,result_pose.y,result_pose.theta);
	return 1;
}
int Pallet_recgnize::legs_recognize(float len,float width,float angle_sech,int clc_cnt,Position & leg1_pose,Position & leg2_pose)////
{

	recognized_start = true;
	pose_recognized = false;
	recognize_cnt = 0;
	max_recog_cnt = clc_cnt;
	pallet_width = width;
	pallet_lenth = len;
	search_angle = angle_sech;
	printf("recv pallet_lenth=%f,pallet_width=%f,search_angle=%f,max_recog_cnt=%d\n",pallet_lenth,pallet_width,search_angle,max_recog_cnt);
	while(recognized_start == true) usleep(10000);
	if(pose_recognized == false) return -1;
//////////////////////////////////根据两个foot的均值，将其赋值给左右腿 
	float xx,yy,thi_foot;
	xx = mean_foot1.x-mean_foot2.x;
	yy = mean_foot1.y-mean_foot2.y;
	float dlen=sqrt(xx*xx+yy*yy);
	float dirx,diry;
	if(mean_foot1.y<mean_foot2.y)
	{
		dirx = -(mean_foot1.y-mean_foot2.y)/dlen;
		diry = (mean_foot1.x-mean_foot2.x)/dlen;
		thi_foot = atan2(diry ,dirx );
		leg1_pose = mean_foot2;
		leg2_pose = mean_foot1;
	}
	else
	{
		/* code */
		dirx = -(mean_foot2.y-mean_foot1.y)/dlen;
		diry = (mean_foot2.x-mean_foot1.x)/dlen;
		thi_foot = atan2(diry ,dirx );
		leg1_pose = mean_foot1;
		leg2_pose = mean_foot2;
	}
	
	printf("leg1_pose pos x=%f,y=%f,theta=%f\n",leg1_pose.x,leg1_pose.y,leg1_pose.theta);
	printf("leg2_pose pos x=%f,y=%f,theta=%f\n",leg2_pose.x,leg2_pose.y,leg2_pose.theta);
	return 1;
}


