#ifndef AGV_STORAGE_H
#define AGV_STORAGE_H

#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <jsoncpp/json/json.h>
#include "common/capture_point.h"


using namespace std;
class storage
{

public:

Json::FastWriter fast_writer;		//仅用于生成网络传输的string
Json::StyledWriter style_writer;	//仅用于打印log

Json::Reader reader;	//用于本地文件转json结构
Json::Value cap_json;
Json::Value route_json;	//用于本机保存
Json::Value chain_json;	//用于本机保存
Json::Value in_json;	//解析传进来的


Json::Value map_json;

void save(char* file_name,Json::Value v);
void insert_cap(CapPoint cap);
void init_db();
void save_routes(std::list<CapRoute> routes);
void save_chains(std::list<RouteChain> chains);
void save_caps(std::list<CapPoint> caps);
//void insert_route(CapRoute route);
/*
void init_route_db();
void modify_route_slot(int routeid,char* name);
void modify_cap_slot(int routeid,int capid,char* str);
void modify_cap_slot(int routeid,int capid,double x,double y,double theta);
void delete_route_slot(int routeid);
void delete_route_cap(int routeid,int capid);
void print_struct();
string get_route_list();*/
int init_map_db();
void modify_map_slot(int id,std::string name);
void delete_map_slot(int id);
void modify_default_map(int id);
string get_map_slot(int id);
string get_map_list();


};

#endif
