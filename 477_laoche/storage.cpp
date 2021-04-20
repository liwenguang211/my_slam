
#include "storage.h"
#include "agv.h"

using namespace std;

//parse	文件转内存结构
//append	数组加元素
void storage::init_db(){
	ifstream ifs("cap.json", ios::binary);
	if (!reader.parse(ifs, cap_json)){	//如果没有配置表文件,则建立
		cout << "no cap" << endl;
		ifs.close();
		string str = style_writer.write(cap_json);
		ofstream ofs("cap.json");
		ofs << str;
		ofs.close();
	}else{	//如果有,则读取
		printf("cap size=%d\n",cap_json.size());
		get_global_agv_instance()->caps.clear();
		for(int i=0;i<cap_json.size();i++){
			CapPoint cap(
			cap_json[i]["id"].asInt(),
			cap_json[i]["x"].asDouble(),
			cap_json[i]["y"].asDouble(),
			cap_json[i]["theta"].asDouble());
			get_global_agv_instance()->caps.push_back(cap);
		}
	}
	ifstream ifs2("route.json", ios::binary);
	if (!reader.parse(ifs2, route_json)){	//如果没有配置表文件,则建立
		cout << "no route" << endl;
		ifs2.close();
		string str = style_writer.write(route_json);
		ofstream ofs("route.json");
		ofs << str;
		ofs.close();
	}else{
		for(int i=0;i<route_json.size();i++){
			CapRoute route;
			route.id=route_json[i]["id"].asInt();
			route.op_id=route_json[i]["op_id"].asInt();
			if(route_json[i]["op_id"].asInt()==0){
				Operation op1,op2,op3,op4,op5;
				op1.type=OP_DELAY;
				op1.delay.sec=3;
				route.op_point.ops.push_back(op1);
				op2.type=OP_LIFT_UP;
				op2.lift.dir=1;
				route.op_point.ops.push_back(op2);
				op3.type=OP_DELAY;
				op3.delay.sec=3;
				route.op_point.ops.push_back(op3);
				op4.type=OP_RADAR;
				op4.radar.mode=1;
				route.op_point.ops.push_back(op4);
				op5.type=OP_MASK;
				op5.mask.onoff=true;
				route.op_point.ops.push_back(op5);
			}else if(route_json[i]["op_id"].asInt()==1){
				Operation op1,op2,op3,op4;
				op1.type=OP_DELAY;
				op1.delay.sec=3;
				route.op_point.ops.push_back(op1);
				op2.type=OP_LIFT_DOWN;
				op2.lift.dir=0;
				route.op_point.ops.push_back(op2);
				op3.type=OP_DELAY;
				op3.delay.sec=3;
				route.op_point.ops.push_back(op3);
				op4.type=OP_MASK;
				op4.mask.onoff=false;
				route.op_point.ops.push_back(op4);
			}
			printf("route%d,id=%d,size=%d,op=%d\n",i,route_json[i]["id"].asInt(),route_json[i]["caps"].size(),route_json[i]["op_id"].asInt());
			for(int j=0;j<route_json[i]["caps"].size();j++){
					CapPoint cap(route_json[i]["caps"][j]["id"].asInt(),
					route_json[i]["caps"][j]["x"].asDouble(),
					route_json[i]["caps"][j]["y"].asDouble(),
					route_json[i]["caps"][j]["theta"].asDouble(),
					route_json[i]["caps"][j]["dir"].asInt(),
					route_json[i]["caps"][j]["collision"].asInt());
					route.cap_points.push_back(cap);
			}
			get_global_agv_instance()->routes.push_back(route);
		}
	}
	ifstream ifs3("chain.json", ios::binary);
	if (!reader.parse(ifs3, chain_json)){	//如果没有配置表文件,则建立
		cout << "no chain" << endl;
		ifs3.close();
		string str = style_writer.write(chain_json);
		ofstream ofs("chain.json");
		ofs << str;
		ofs.close();
	}else{
		for(int i=0;i<chain_json.size();i++){
			RouteChain chain;
			chain.id=chain_json[i]["id"].asInt();
			
			printf("chain%d,id=%d,size=%d\n",i,chain_json[i]["id"].asInt(),chain_json[i]["routes"].size());
			for(int j=0;j<chain_json[i]["routes"].size();j++){
					CapRoute route;
					int route_id=chain_json[i]["routes"][j]["id"].asInt();
					for(CapRoute &r : get_global_agv_instance()->routes) {
						if(r.id==route_id){
							CapRoute route=r;	//todo  test
							chain.routes.push_back(route);
							printf("	route id=%d,caps.size=%d\n",route_id,route.cap_points.size());
						}
					}
			}
			get_global_agv_instance()->chains.push_back(chain);
		}
	}
}
void storage::insert_cap(CapPoint cap){
	int exist=0;
	int i=0;
	for(i=0;i<cap_json.size();i++){
		if(cap_json[i]["id"].asInt()==cap.id){
			exist=1;
			break;
		}
	}
	if(exist){
		cap_json[i]["x"]=cap.x;
		cap_json[i]["y"]=cap.y;
		cap_json[i]["theta"]=cap.theta;
		cap_json[i]["dir"]=cap.dir;
		cap_json[i]["collision"]=cap.collision;
	}else{
		Json::Value capj;
		capj["id"]=cap.id;
		capj["x"]=cap.x;
		capj["y"]=cap.y;
		capj["theta"]=cap.theta;
		capj["dir"]=cap.dir;
		capj["collision"]=cap.collision;
		cap_json.append(capj);
	}
	save("cap.json",cap_json);
}


void storage::save_caps(std::list<CapPoint> caps){
	
	//printf();
	cap_json.clear();
	for(CapPoint &cap : caps) {	
		Json::Value capj;
		capj["id"]=cap.id;
		capj["x"]=cap.x;
		capj["y"]=cap.y;
		capj["theta"]=cap.theta;
		cap_json.append(capj);
	}
	
	save("cap.json",cap_json);
}
void storage::save_routes(std::list<CapRoute> routes){
	route_json.clear();
	for(CapRoute &route : routes) {	
		Json::Value routej;
		routej["id"]=route.id;
		routej["op_id"]=route.op_id;
		printf("save_routes,id=%d,op=%d\n",route.id,route.op_id);
		Json::Value routearray;
		int i=0;
		for(CapPoint &cap : route.cap_points) {	
			Json::Value capj;
			capj["id"]=cap.id;
			capj["x"]=cap.x;
			capj["y"]=cap.y;
			capj["theta"]=cap.theta;
			capj["dir"]=cap.dir;
			capj["collision"]=cap.collision;
			routearray[i]=capj;
			i++;
		}
		routej["caps"]=routearray;
		route_json.append(routej);
	}
	
	save("route.json",route_json);
}
void storage::save_chains(std::list<RouteChain> chains){
	chain_json.clear();
	for(RouteChain &chain : chains) {	
		Json::Value chainj;
		chainj["id"]=chain.id;
		Json::Value chainarray;
		int i=0;
		for(CapRoute &route : chain.routes) {	
			Json::Value routej;
			routej["id"]=route.id;
			chainarray[i]=routej;
			i++;
		}
		chainj["routes"]=chainarray;
		chain_json.append(chainj);
	}
	
	save("chain.json",chain_json);
}

void storage::save(char* file_name,Json::Value v){
	string str = style_writer.write(v);
	ofstream ofs(file_name,ios::trunc);
	ofs << str;
	ofs.close();
}

/*


void storage::init_route_db(){//route.json
	ifstream ifs("route.json", ios::binary);
	if (!reader.parse(ifs, route_json)){	//如果没有配置表文件,则建立
		cout << "no route" << endl;
		ifs.close();
		string str = style_writer.write(route_json);
		ofstream ofs("route.json");
		ofs << str;
		ofs.close();
	}else{
		for (int i = 0; i < ROUTE_SLOT_NUM;i++){
			char routen[10];
			sprintf(routen,"route%d",i);
			
			if(route_json.isMember(routen)){
				get_global_agv_instance()->r[i].inuse=1;
				for(int j=0;j<CAP_SLOT_NUM;j++){
					char capn[10];
					sprintf(capn,"cap%d",j);
					if(route_json[routen].isMember(capn)){
						get_global_agv_instance()->r[i].cap[j].inuse=1;
						get_global_agv_instance()->r[i].cap[j].x=route_json[routen][capn]["x"].asDouble();
						get_global_agv_instance()->r[i].cap[j].y=route_json[routen][capn]["y"].asDouble();
						get_global_agv_instance()->r[i].cap[j].theta=route_json[routen][capn]["theta"].asDouble();
						get_global_agv_instance()->r[i].cap[j].max_speed=route_json[routen][capn]["max_speed"].asInt();
						get_global_agv_instance()->r[i].cap[j].action=route_json[routen][capn]["action"].asInt();
						get_global_agv_instance()->r[i].cap[j].stay_time=route_json[routen][capn]["stay_time"].asInt();
						get_global_agv_instance()->r[i].cap[j].collision_1=route_json[routen][capn]["collision_1"].asInt();
						get_global_agv_instance()->r[i].cap[j].collision_2=route_json[routen][capn]["collision_2"].asInt();
						get_global_agv_instance()->r[i].cap[j].collision_3=route_json[routen][capn]["collision_3"].asInt();
					}
				}
			}
		}
		ifs.close();
	}
}

void storage::modify_route_slot(int routeid,char* name){
	
	get_global_agv_instance()->r[routeid].inuse=1;//
	memcpy(get_global_agv_instance()->r[routeid].name,name,strlen(name));
	
	char routen[10];
	sprintf(routen,"route%d",routeid);
	route_json[routen]["name"]=name;
	
	string str = style_writer.write(route_json);
	ofstream ofs("route.json");
	ofs << str;
	ofs.close();
}
void storage::modify_cap_slot(int routeid,int capid,char* str){
	if(get_global_agv_instance()->r[routeid].inuse==1){
		get_global_agv_instance()->r[routeid].cap[capid].inuse=1;
		char routen[10];
		sprintf(routen,"route%d",routeid);
		char capn[10];
		sprintf(capn,"cap%d",capid);
		
		
		if (reader.parse(str, in_json)){
			
			if(in_json.isMember("x")){
				get_global_agv_instance()->r[routeid].cap[capid].x=in_json["x"].asDouble();
				route_json[routen][capn]["x"]=in_json["x"].asDouble();
			}
			
			if(in_json.isMember("y")){
				get_global_agv_instance()->r[routeid].cap[capid].y=in_json["y"].asDouble();
				route_json[routen][capn]["y"]=in_json["y"].asDouble();
			}
			if(in_json.isMember("theta")){
				get_global_agv_instance()->r[routeid].cap[capid].theta=in_json["theta"].asDouble();
				route_json[routen][capn]["theta"]=in_json["theta"].asDouble();
			}
			if(in_json.isMember("max_speed")){
				get_global_agv_instance()->r[routeid].cap[capid].max_speed=in_json["max_speed"].asInt();
				route_json[routen][capn]["max_speed"]=in_json["max_speed"].asInt();
				
				printf("in_json.isMember(max_speed)\n");
			}
			if(in_json.isMember("action")){
				get_global_agv_instance()->r[routeid].cap[capid].action=in_json["action"].asInt();
				route_json[routen][capn]["action"]=in_json["action"].asInt();
			}
			if(in_json.isMember("stay_time")){
				get_global_agv_instance()->r[routeid].cap[capid].stay_time=in_json["stay_time"].asInt();
				route_json[routen][capn]["stay_time"]=in_json["stay_time"].asInt();
			}
			if(in_json.isMember("collision_1")){
				get_global_agv_instance()->r[routeid].cap[capid].collision_1=in_json["collision_1"].asInt();
				route_json[routen][capn]["collision_1"]=in_json["collision_1"].asInt();
			}
			if(in_json.isMember("collision_2")){
				get_global_agv_instance()->r[routeid].cap[capid].collision_2=in_json["collision_2"].asInt();
				route_json[routen][capn]["collision_2"]=in_json["collision_2"].asInt();
			}
			if(in_json.isMember("collision_3")){
				get_global_agv_instance()->r[routeid].cap[capid].collision_3=in_json["collision_3"].asInt();
				route_json[routen][capn]["collision_3"]=in_json["collision_3"].asInt();
			}
		}else{
		}
	}
	
	string strs = style_writer.write(route_json);
	ofstream ofs("route.json");
	ofs << strs;
	ofs.close();
}
void storage::modify_cap_slot(int routeid,int capid,double x,double y,double theta){
	if(get_global_agv_instance()->r[routeid].inuse==1){
		get_global_agv_instance()->r[routeid].cap[capid].inuse=1;
		char routen[10];
		sprintf(routen,"route%d",routeid);
		char capn[10];
		sprintf(capn,"cap%d",capid);
		get_global_agv_instance()->r[routeid].cap[capid].x=in_json["x"].asDouble();
		route_json[routen][capn]["x"]=in_json["x"].asDouble();
		get_global_agv_instance()->r[routeid].cap[capid].y=in_json["y"].asDouble();
		route_json[routen][capn]["y"]=in_json["y"].asDouble();
		get_global_agv_instance()->r[routeid].cap[capid].theta=in_json["theta"].asDouble();
		route_json[routen][capn]["theta"]=in_json["theta"].asDouble();
		
	}
	
	string strs = style_writer.write(route_json);
	ofstream ofs("route.json");
	ofs << strs;
	ofs.close();
}
void storage::delete_route_slot(int routeid){
	
	get_global_agv_instance()->r[routeid].inuse=0;
	char routen[10];
	sprintf(routen,"route%d",routeid);
	route_json.removeMember(routen);
	
	string str = style_writer.write(route_json);
	ofstream ofs("route.json");
	ofs << str;
	ofs.close();
}
void storage::delete_route_cap(int routeid,int capid){
	get_global_agv_instance()->r[routeid].cap[capid].inuse=0;
	
	char routen[10];
	sprintf(routen,"route%d",routeid);
	
	char capn[10];
	sprintf(capn,"cap%d",capid);
	route_json[routen].removeMember(capn);
	
	string str = style_writer.write(route_json);
	ofstream ofs("route.json");
	ofs << str;
	ofs.close();
}
void storage::print_struct(){
	for(int i=0;i<ROUTE_SLOT_NUM;i++){
		printf("-----------------------\n");
		printf("r[%d].inuse=%d,name=%s\n",i,get_global_agv_instance()->r[i].inuse,get_global_agv_instance()->r[i].name);
		if(get_global_agv_instance()->r[i].inuse){
			for(int j=0;j<CAP_SLOT_NUM;j++){
				if(get_global_agv_instance()->r[i].cap[j].inuse){
					printf("route[%d].cap[%d]=%d,%f,%f,%f\n",i,j,
					get_global_agv_instance()->r[i].cap[j].id,
					get_global_agv_instance()->r[i].cap[j].x,
					get_global_agv_instance()->r[i].cap[j].y,
					get_global_agv_instance()->r[i].cap[j].theta);
				}
			}
		}
	}
}

string storage::get_route_list(){
	return fast_writer.write(route_json);
}
*/
int storage::init_map_db(){
	char slot_id[10];
	ifstream ifs("local.json", ios::binary);
	if (!reader.parse(ifs, map_json)){	//如果没有配置表文件,则建立
		cout << "no map" << endl;
		ifs.close();
		
		for(int i=1;i<=40;i++){
			sprintf(slot_id,"%d",i);
			map_json[slot_id]="--";
		}
		map_json["default"]=255;
		string str = style_writer.write(map_json);
		ofstream ofs("local.json");
		ofs << str;
		ofs.close();
	}else{
		cout << "have map\n";
		for(int i=1;i<=40;i++){
			sprintf(slot_id,"%d",i);
			if(map_json[slot_id].asString()!="--"){
				cout << "map" << slot_id << ":" << map_json[slot_id].asString() << endl;
			}
		}
		cout << "default map:" << map_json["default"].asString() << endl;
		ifs.close();
		cout << endl;
	}
	return map_json["default"].asInt();
}
void storage::modify_map_slot(int id,std::string name){
	char slot_id[10];
	sprintf(slot_id,"%d",id);
	map_json[slot_id]= name;
	string str = style_writer.write(map_json);
	ofstream ofs("local.json");
	ofs << str;
	ofs.close();
}
void storage::modify_default_map(int id){
	map_json["default"]= id;
	string str = style_writer.write(map_json);
	ofstream ofs("local.json");
	ofs << str;
	ofs.close();
}
void storage::delete_map_slot(int id){
	
	char slot_id[10];
	sprintf(slot_id,"%d",id);
	map_json[slot_id]= "--";
	
	string str = style_writer.write(map_json);
	ofstream ofs("local.json");
	ofs << str;
	ofs.close();
}
string storage::get_map_slot(int id){
	char slot_id[10];
	sprintf(slot_id,"%d",id);
	return map_json[slot_id].asString();
}

string storage::get_map_list(){
	return fast_writer.write(map_json);
}
