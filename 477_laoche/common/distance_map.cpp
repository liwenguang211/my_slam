
#include <cmath>
#include <stdio.h>

#include "distance_map.h"
#include "huffman.h"
#include "configuration.h"

DistanceMap::DistanceMap(GlobalCoordinateMap *grid_map) : DistanceMap(grid_map, get_configs()->get_int("distance_map", "range", nullptr)) {
    
}

DistanceMap::DistanceMap(GlobalCoordinateMap *grid_map, int max_distance) {
    
    GET_CONFIG_DOUBLE(occupied, "map", "occupied_threshold");
    int range = max_distance;
    map_info = grid_map->get_info();
    int size = map_info.width * map_info.height;

    double sigma_hit = 0.07;
    double rang_max = 0.05;
    double z_hit = 0.95;
    double z_rand = 0.002;
    double z_hit_denom = 2 * sigma_hit * sigma_hit;
    double z_rand_mult = 1.0 / rang_max;
    double pz = 0.;
    double pi = 3.1415926535;

    distance_data.reserve(size);
    distance_data.assign(size, -1.);

    int x, y;
    int start_x, end_x, start_y, end_y;
    for(int i = 0;i < size;i++) {
        map_info.array_index_to_map_xy(i, &x, &y);
        double d = grid_map->get(map_info.map_x_to_global_x(x), map_info.map_y_to_global_y(y));
        if(d < occupied) {
            continue;
        }
        start_x = ((x - range < 0) ? 0 : (x - range));
        end_x = ((x + range + 1 > map_info.width) ? map_info.width : (x + range + 1));
        start_y = ((y - range < 0) ? 0 : (y - range));
        end_y = ((y + range + 1 > map_info.height) ? map_info.height : (y + range + 1));
        for(int a = start_x; a < end_x;a++) {
            for(int b = start_y; b < end_y;b++) {
                double distance_square = (a - x) * (a - x) + (b - y) * (b - y);
                int i2 = map_info.map_xy_to_array_index(a, b);
                if(distance_data[i2] < 0 || distance_data[i2] > distance_square) {
                    distance_data[i2] = distance_square;
                }
            }
        }
    }

    double max = 0.;
    for(int i = 0;i < size; i++) {
        if(distance_data[i] >= 0) {
            pz = 0;
            distance_data[i] = sqrt(distance_data[i]) * map_info.resolution;
            pz += z_hit * (1/(sqrt(2*pi)*sigma_hit)) * exp(-(distance_data[i] * distance_data[i])/z_hit_denom);
            // pz += z_rand * z_rand_mult;
            distance_data[i] = pz;
            if(pz > max) {
                max = pz;
            }
        }
    }
    
    for(int i = 0;i < size; i++) {
        if(distance_data[i] > 0) {
            distance_data[i] = distance_data[i] / max;
        }
    }
}

double DistanceMap::get(double x, double y) {
    int x_map = map_info.global_x_to_map_x(x);
    int y_map = map_info.global_y_to_map_y(y);
    if(x_map < 0 || y_map < 0 || x_map >= map_info.width || y_map >= map_info.height) {
        return -1;
    }
    return distance_data[map_info.map_xy_to_array_index(x_map, y_map)];
}

int DistanceMap::to_char_array(std::vector<char> *output) {
    std::vector<char> char_data;
    std::vector<char> compressed;
    int dsize = map_info.width * map_info.height;
    char_data.reserve(dsize);
    for(int i = 0;i < dsize;i++) {
        double dist = distance_data[i];
        if(dist < 0) {
            char_data.push_back(100);
        }
        else {
            int d = (int)(dist * 100.);
            if(d > 100) {
                d = 100;
            }
            char_data.push_back(d);
        }
    }

    dsize = huffman_compress(&char_data, &compressed);
    int size = dsize + sizeof(MapInfo);
    output->reserve(output->size() + size + sizeof(int));
    char *p_size = (char *)(&size);
    for (int i = 0; i < sizeof(int); i++)
    {
        output->push_back(p_size[i]);
    }
    char * p_info = (char*)(& map_info);
    for(int i = 0;i < sizeof(MapInfo);i++) {
        output->push_back(p_info[i]);
    }
    for(int i = 0;i < dsize;i++) {
        output->push_back(compressed[i]);
    }
    return size;
}

