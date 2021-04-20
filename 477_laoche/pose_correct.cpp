
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <Eigen/Core>

#include "pose_correct.h"
class RTCostFunctor2D {//////构建计算两组空间点的旋转平移分量的残差函数
public:
	RTCostFunctor2D(double xo, double yo, double xd, double yd)
		: xorig(xo), yorig(yo), xdest(xd), ydest(yd) {}

	template <typename T>
	bool operator()(const T* const pose, T* residual) const {
		residual[0] = xorig*cos(pose[2]) - sin(pose[2])*yorig + pose[0] - xdest;
		residual[1] = sin(pose[2])*xorig + cos(pose[2])*yorig + pose[1] - ydest;
		return true;
	}

private:
	RTCostFunctor2D(const RTCostFunctor2D&) = delete;
	RTCostFunctor2D& operator=(const RTCostFunctor2D&) =
		delete;

	const double xorig;
	const double yorig;
	const double xdest;
	const double ydest;
};

class CalAngFunctor2D {//////构建计算两组空间点的角度偏差分量的残差函数
public:
	CalAngFunctor2D(double ao, double ad)
		: Aorig(ao), Adest(ad) {}

	template <typename T>
	bool operator()(const T* const delt_a, T* residual) const {
		residual[0] = Adest - Aorig - delt_a;
		return true;
	}

private:
	CalAngFunctor2D(const CalAngFunctor2D&) = delete;
	CalAngFunctor2D& operator=(const CalAngFunctor2D&) =
		delete;

	const double Aorig;
	const double Adest;

};
//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------

Position raw_pos_correct(raw_pos_for_correct & rawpos, int num)
{
	double opt_buf[3];
	Position ori_odom = rawpos.odom_raw_pos[0];
	Position ori_scan = rawpos.scan_match_raw_pos[0];
	for (int i = 0; i < 10; i++)
	{///////对数据的增量进行处理，输出结果同样是增量
		//printf("输入的原始odo数据%f=%f=%f===scan数据%f=%f=%f\n",rawpos.odom_raw_pos[i].x,rawpos.odom_raw_pos[i].y,rawpos.odom_raw_pos[i].theta,rawpos.scan_match_raw_pos[i].x,rawpos.scan_match_raw_pos[i].y,rawpos.scan_match_raw_pos[i].theta);
		//rawpos.odom_raw_pos[i] = rawpos.odom_raw_pos[i] - ori_odom;
		//rawpos.scan_match_raw_pos[i] = rawpos.scan_match_raw_pos[i] - ori_odom;
	}
	////////首先计算一个初始位姿数据，用于cere优化的初值
	/*double thetao = atan2(rawpos.odom_raw_pos[9].y - rawpos.odom_raw_pos[0].y, rawpos.odom_raw_pos[9].x - rawpos.odom_raw_pos[0].x);
	double thetad = atan2(rawpos.scan_match_raw_pos[9].y - rawpos.scan_match_raw_pos[0].y, rawpos.scan_match_raw_pos[9].x - rawpos.scan_match_raw_pos[0].x);
	double del_tehta = thetad - thetao;
	///////////////////////////////
	double cdel_theta = cos(del_tehta);
	double sdel_theta = sin(del_tehta);
	double mid_xo = (rawpos.odom_raw_pos[9].x + rawpos.odom_raw_pos[0].x)*0.5;
	double mid_yo = (rawpos.odom_raw_pos[9].y + rawpos.odom_raw_pos[0].y)*0.5;

	double rmid_xo = cdel_theta *mid_xo - sdel_theta *mid_yo;
	double rmid_yo = sdel_theta *mid_xo + cdel_theta *mid_yo;

	double delx = (rawpos.scan_match_raw_pos[9].x + rawpos.scan_match_raw_pos[0].x)*0.5 - rmid_xo;
	double dely = (rawpos.scan_match_raw_pos[9].y + rawpos.scan_match_raw_pos[0].y)*0.5 - rmid_yo;;
	double delt = del_tehta;*/

	double delx = ori_scan.x-ori_odom.x;
	double dely = ori_scan.y-ori_odom.y ;
	double delt = ori_scan.theta-ori_odom.theta;

	opt_buf[0] = delx;
	opt_buf[1] = dely;
	opt_buf[2] = delt;
	printf("From (%.3f, %.3f, %.3f)\n", delx, dely, delt);
	ceres::Solver::Options options;
	options.max_num_iterations = 10;
	options.linear_solver_type = ceres::DENSE_QR;
	ceres::Problem problem;
	double xo, yo, xd, yd, ao, ad;
	for (int j = 0; j < num; ++j)
	{
		xo = rawpos.odom_raw_pos[j].x;
		yo = rawpos.odom_raw_pos[j].y;

		xd = rawpos.scan_match_raw_pos[j].x;
		yd = rawpos.scan_match_raw_pos[j].y;

		ao = rawpos.odom_raw_pos[j].theta;
		ad = rawpos.scan_match_raw_pos[j].theta;
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RTCostFunctor2D, 2, 3>(
			new RTCostFunctor2D(xo, yo, xd, yd)),
			nullptr,
			opt_buf);

	}

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	double mid_xo ;
	double mid_yo ;
	double cdel_theta;
	double sdel_theta;

	delt = opt_buf[2];
	delx = opt_buf[0];
	dely = opt_buf[1];
	cdel_theta = cos(delt);
	sdel_theta = sin(delt);
	mid_xo = rawpos.odom_raw_pos[9].x;
	mid_yo = rawpos.odom_raw_pos[9].y;
	double xdest = cdel_theta *mid_xo - sdel_theta *mid_yo + delx ;
	double ydest = sdel_theta *mid_xo + cdel_theta *mid_yo + dely ;
	double thedest = rawpos.scan_match_raw_pos[9].theta;
	// std::cout << summary.BriefReport() << "\n";
	// printf("From (%.3f, %.3f, %.3f)\n", init.x, init.y, init.theta);
	printf("To   (%.3f, %.3f, %.3f)\n", delx, dely, delt);
	printf("目标位置修正为  (%.3f, %.3f, %.3f)\n", xdest, ydest, delt);


	return Position(rawpos.scan_match_raw_pos[9].timestamp, xdest, ydest, thedest );
}


