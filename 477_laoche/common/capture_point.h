#ifndef CAPTURE_POINT_H
#define CAPTURE_POINT_H

#include <list>
#include "byte_data.h"


#define ROUTE_SLOT_NUM	20
#define CAP_SLOT_NUM	20
#define MAP_SLOT_NUM	40

//带管理的导航接口
typedef struct cap{
	int id;
	int inuse;
	double x;
	double y;
	double theta;
	int max_speed;
	int action;
	int stay_time;
	int collision_1;
	int collision_2;
	int collision_3;
}CAP;

typedef struct route{
	int id;
	int inuse;
	int cap_num;//这个?
	char name[100];
	CAP cap[CAP_SLOT_NUM];
}ROUTE;

//300/h,1:12
//7000,370,1000

#define OP_DELAY		1
#define OP_RADAR		3
#define OP_MASK			4
#define OP_LIFT_UP		5
#define OP_LIFT_DOWN	6
#define OP_LIFT_ROTATE	7
#define OP_WAIT_DI		8


struct operation_delay{
    int sec;
};
struct operation_lift{
    int dir;
};
struct operation_analyzed_radar{
	int mode;
};
struct operation_mask_switch{
	bool onoff;
};
class Operation{
public:
	int type;
	struct operation_delay 	delay;
	struct operation_lift	lift;
	struct operation_analyzed_radar 	radar;
	struct operation_mask_switch 	mask;
	char* get_op_name(){
		switch(type){
			case OP_DELAY:		return "OP_DELAY";
			case OP_RADAR:		return "OP_RADAR";
			case OP_MASK:		return "OP_MASK";
			case OP_LIFT_UP:	return "OP_LIFT_UP";
			case OP_LIFT_DOWN:	return "OP_LIFT_DOWN";
			case OP_LIFT_ROTATE:return "OP_LIFT_ROTATE";
			case OP_WAIT_DI:	return "OP_WAIT_DI";
			default:			return "";
		}
	}
};

/*

	get_global_storage()->save_caps(get_global_agv_instance()->caps);
	get_global_storage()->save_routes(get_global_agv_instance()->routes);

*/
class OperationPoint{
public:
	int max_speed;
	std::list<Operation>	ops;
};

//旧导航接口
class CapPoint {
public:
    CapPoint(int id,double x, double y) : id(id),x(x), y(y) ,theta(0),dir(0){};
    CapPoint(int id,double x, double y,double theta) : id(id),x(x), y(y),theta(theta),dir(0),collision(1) {};
    CapPoint(int id,double x, double y,double theta,int dir,int collision) : id(id),x(x), y(y),theta(theta) ,dir(dir),collision(collision){};
    CapPoint() : x(0.), y(0.) {};
    double x;
    double y;
	double theta;
	int dir;
	int collision;
	int id;
};

class CapRoute {
public:
    std::list<CapPoint> cap_points;
	OperationPoint	op_point;
	int op_id;
	int id;
};
class RouteChain {
public:
    std::list<CapRoute> routes;
	int id;
};

class Action_clm {
public:
    Action_clm(double x, double y) : x(x), y(y) ,type(1) {};
    Action_clm(double angle) : angle(angle) ,type(0) {};
	
    Action_clm(double x1,double y1,double theta1,double x2,double y2,double theta2,double x, double y) 
	: from_x(x1),from_y(y1),from_theta(theta1),to_x(x2),to_y(y2),to_theta(theta2),x(x), y(y) ,angle(0),type(1) {};
	
    Action_clm(double x1,double y1,double theta1,double x2,double y2,double theta2,double x, double y,int dir) 
	: from_x(x1),from_y(y1),from_theta(theta1),to_x(x2),to_y(y2),to_theta(theta2),x(x), y(y) ,angle(0),type(1),dir(dir) {};
	
    Action_clm(double x1,double y1,double theta1,double x2,double y2,double theta2,double angle) 
	: from_x(x1),from_y(y1),from_theta(theta1),to_x(x2),to_y(y2),to_theta(theta2),x(0),y(0),angle(angle) ,type(0){};
    double from_x;
    double from_y;
	double from_theta;
    double to_x;
    double to_y;
	double to_theta;
	
	
    double x;
    double y;
	double angle;
	int type;	//1直行0转向
	int dir;	//仅用于直行模式的方向:0正向1反向
};
/*
class CheckPoint {
public:
    CheckPoint(double x, double y) : x(x), y(y) {};
    CheckPoint() : x(0.), y(0.) {};
    double x;
    double y;
};
*/
struct CapturePointExtraInfo {
    int     test;
};

class CapturePoint : public Serializable {
public:
    CapturePoint(int index, char* name) : index(index) {
        int len = 0;
        for(len = 0;name[len] != 0;len++);
        this->name = new char[++len];
        for(int i = 0;i < len;i++) {
            this->name[i] = name[i];
        }
    };
    CapturePoint(char *buf, int size) {
        from_char_array(buf, size);
    };
    CapturePoint(const CapturePoint& p2) : CapturePoint(p2.index, p2.name) {
        x = p2.x;
        y = p2.y;
        theta = p2.theta;
        extra = p2.extra;
    };
    ~CapturePoint() {if(name) delete name;}

    virtual void from_char_array(char *buf, int size);
    virtual void to_char_array(std::vector<char> *output);

    void rename(char *new_name);

    int     index;
    char *  name = nullptr;
    double  x;
    double  y;
    double  theta;
    struct CapturePointExtraInfo    extra; //额外信息
};

#endif

