
#ifndef AGV_LOW_RESOLUTION_H
#define AGV_LOW_RESOLUTION_H

#include <vector>

#include "common/simple_grid_map.h"

class Low_resolution : public GlobalCoordinateMap {
public:
	//Low_resolution(GlobalCoordinateMap *grid_map);
	Low_resolution(SimpleGridMap * map,double thres);
	~Low_resolution() {};

    virtual double get(double x, double y);
    virtual MapInfo get_info() {return map_info;};

	SimpleGridMap* low_reso_map();
	float threshold;
	std::vector<char> datas;
    MapInfo map_info;
	LandmarkMap landmark;
	SimpleGridMap *gridmap_ret;
	std::vector<CapturePoint> cap_points;

};


#endif
