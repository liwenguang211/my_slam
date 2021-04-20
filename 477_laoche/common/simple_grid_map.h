
#ifndef AGV_SIMPLE_GRID_MAP_H
#define AGV_SIMPLE_GRID_MAP_H

#include <vector>

#include "grid_map.h"
#include "landmark_map.h"
#include "capture_point.h"
#include "byte_data.h"

class SimpleGridMap : public GlobalCoordinateMap , public Serializable {
public:
    SimpleGridMap(MapInfo & info);
    SimpleGridMap(SimpleGridMap & map);
    SimpleGridMap(char * data_buf, int buf_size);
    ~SimpleGridMap();
    
    virtual double get(double x, double y);
    virtual MapInfo get_info() {
        return map_info;
    };

    virtual void to_char_array(std::vector<char> *output);
    virtual void from_char_array(char *buf, int size);

    LandmarkMap *get_landmark_map() {return &landmark;};
    
    MapInfo map_info;
    std::vector<char> datas;
    pthread_mutex_t mutex;

    LandmarkMap landmark;

    std::vector<CapturePoint> cap_points;

    void binarization(double threshold);
};

#endif
