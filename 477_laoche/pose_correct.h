#ifndef AGV_POS_CORRECT_H
#define AGV_POS_CORRECT_H

#include <vector>
#include <Eigen/Core>

#include "common/grid_map.h"
#include "common/sensor_data.h"

typedef struct {
	Position odom_raw_pos[10];
	Position scan_match_raw_pos[10];
} raw_pos_for_correct;


Position raw_pos_correct(raw_pos_for_correct & rawpos,int num);


#endif
