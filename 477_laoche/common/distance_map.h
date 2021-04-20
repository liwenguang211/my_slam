
#ifndef AGV_DISTANCE_MAP_H
#define AGV_DISTANCE_MAP_H

#include <vector>

#include "grid_map.h"

class DistanceMap : public GlobalCoordinateMap {
public:
    DistanceMap(GlobalCoordinateMap *grid_map);
    DistanceMap(GlobalCoordinateMap *grid_map, int max_distance);
    ~DistanceMap() {};

    virtual double get(double x, double y);
    virtual MapInfo get_info() {return map_info;};

    int to_char_array(std::vector<char> *output);
    
private:
    MapInfo map_info;
    std::vector<double> distance_data;

};


#endif
