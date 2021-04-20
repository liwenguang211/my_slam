#ifndef AGV_SCAN_MATCHING_H
#define AGV_SCAN_MATCHING_H

#include <vector>
#include <Eigen/Core>

#include "common/grid_map.h"
#include "common/sensor_data.h"

class GridArrayAdapter {
public:
    enum { DATA_DIMENSION = 1 };

    GridArrayAdapter(GlobalCoordinateMap * grid) : grid_(grid) {
        info = grid_->get_info();
    }

    void GetValue(int row, int column, double* value) const {
        if (row < 0 || column < 0 || row >= info.width || column >= info.height) {
            *value = 1.;
        } else {
            double d = grid_->get((row * info.resolution + info.origen_x), (column * info.resolution + info.origen_y));
            if(d < 0) {
                d = 0;
            }
            *value = 1 - d;
        }
    }

    int NumRows() const {
    return info.width;
    }

    int NumCols() const {
    return info.height;
    }

private:
    MapInfo info;
    GlobalCoordinateMap * grid_;
};

class ScanMatchingOptions {
public:
    double      occupied_space_cost_factor      = 1.;
    double      translation_delta_cost_factor   = 10.;
    double      rotation_delta_cost_factor      = 20.;

    int         max_num_iterations      = 10;
    int         num_threads             = 2;

    bool        use_nonmonotonic_steps  = false;

};

Position scan_matching(Position & init, GlobalCoordinateMap *map, PointCloudData &range_data, ScanMatchingOptions &option);


#endif
