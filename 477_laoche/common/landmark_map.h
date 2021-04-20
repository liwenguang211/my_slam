#ifndef LANDMARK_MAP_H
#define LANDMARK_MAP_H

#include <map>
#include <set>
#include <vector>
#include <list>

#include "grid_map.h"
#include "byte_data.h"

class LandmarkMap : public GlobalCoordinateMap , public Serializable {
public:
    LandmarkMap() {};
    LandmarkMap(MapInfo &info) : info(info) {};
    ~LandmarkMap() {};
    
    void insert(double x, double y);
    void insert(int index);

    void set_mapinfo(MapInfo &i);

    void gridding();

    void remove_noise();

    virtual void to_char_array(std::vector<char> *output);
    virtual void from_char_array(char *buf, int size);

public:
    virtual double get(double x, double y);
    virtual MapInfo get_info() {return info; };

private:
    MapInfo info;
    bool has_init = true; //unused

    std::set<int> points;
    std::vector<char> bit_array;
    bool is_grid = false;
};

class LandmarkMapGenerator {
public:
    LandmarkMapGenerator(double resolution) : resolution(resolution) {};
    ~LandmarkMapGenerator() {};

    void add_point(double x, double y);
    LandmarkMap to_landmark_map();

private:
    double resolution;
    std::map<long, int> grid_data;

    int x_min = 0x7FFFFFFF, x_max = -0x7FFFFFFF;
    int y_min = 0x7FFFFFFF, y_max = -0x7FFFFFFF;
};

#endif
