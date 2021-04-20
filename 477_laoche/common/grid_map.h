#ifndef CARTO_GRID_MAP
#define CARTO_GRID_MAP

#include <pthread.h>
#include <vector>

using std::vector;


/**
 * 储存一个绘制好的格栅图数据
 */
class MapInfo {
public:
    int                 width       = 0;    //宽
    int                 height      = 0;    //高
    double              origen_x    = 0;    //左上角x坐标
    double              origen_y    = 0;    //左上角y坐标
    double              resolution  = 0;    //分辨率，单位m

    int global_x_to_map_x(double x) {
        return (int)((x - origen_x) / resolution);
    }
    int global_y_to_map_y(double y) {
        return (int)((y - origen_y) / resolution);
    }
    int map_xy_to_array_index(int x, int y) {
        return (y*width + x);
    }
    void array_index_to_map_xy(int index, int *px, int *py) {
        *py = index / width;
        *px = index % width;
    }
    double map_x_to_global_x(int x) {
        return (x * resolution + origen_x);
    }
    double map_y_to_global_y(int y) {
        return (y * resolution + origen_y);
    }
};

class GlobalCoordinateMap {
public:
    virtual double get(double x, double y) = 0;
    virtual MapInfo get_info() = 0;
};

#endif

