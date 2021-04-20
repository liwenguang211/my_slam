
#include "landmark_map.h"
#include "configuration.h"
#include <stdio.h>

void LandmarkMap::set_mapinfo(MapInfo &i) {
    info = i;
    points.clear();
    has_init = true;
}

void LandmarkMap::insert(double x, double y) {
    if(!has_init) {
        return;
    }
    int x1 = info.global_x_to_map_x(x);
    int y1 = info.global_y_to_map_y(y);
    insert(info.map_xy_to_array_index(x1, y1));
}

void LandmarkMap::insert(int index) {
    if(!has_init) {
        return;
    }
    if(index < 0 || index > info.height * info.width) {
        printf("Landmark insert error\n");
        return ;
    }
    if(points.find(index) == points.end()) {
        points.insert(index);
    }
    if(is_grid) {
        is_grid = false;
        bit_array.clear();
    }
}

double LandmarkMap::get(double x, double y) {
    if(!has_init) {
        return 0.;
    }
    int x_map = info.global_x_to_map_x(x);
    int y_map = info.global_y_to_map_y(y);
    if(x_map < 0 || y_map < 0 || x_map >= info.width || y_map >= info.height) {
        return 0.;
    }
    int index = info.map_xy_to_array_index(x_map, y_map);
    
    if(!is_grid) {
        if(points.find(index) != points.end()) {
            return 1.;
        }
    }
    else {
        if(bit_array[index / 8] & (1 << (index % 8)) != 0) {
            return 1.;
        }
    }

    return 0.;
}

void LandmarkMap::gridding() {
    if(is_grid) {
        bit_array.clear();
    }
    int grid_count = (info.width / info.resolution) * (info.height / info.resolution);
    bit_array.assign(grid_count / 8 + 1, 0);

    int index, bit_index, bit_offset;
    for(auto i = points.begin(); i != points.end(); i++)
    {
        index = *i;
        bit_index = index / 8;
        bit_offset = index % 8;
        bit_array[bit_index] = bit_array[bit_index] | (1 << bit_offset);
    }

    is_grid = true;
}

void LandmarkMap::remove_noise() {
    std::set<int> old = points;
    points.clear();
    is_grid = false;
    for(int index : old) {
        int x, y;
        info.array_index_to_map_xy(index, &x, &y);
        int sum = 0;
        for(int m = x - 1; m <= x + 1; m++) {
            for(int n = y - 1; n <= y + 1; n++) {
                if(m < 0 || m >= info.width || n < 0 || n >= info.height) {
                    continue;
                }
                int target = info.map_xy_to_array_index(m, n);
                auto search = old.find(target);
                if(search != old.end()) {
                    sum ++;
                }
            }
        }
        if(sum > 2) {
            points.insert(index);
        }
    }
}

void LandmarkMap::to_char_array(std::vector<char> *output) {
    std::vector<char> datas;
    datas.reserve(points.size() * sizeof(int));
    int d = 0;
    char *p = (char*)(&d);
    for(auto i = points.begin(); i != points.end(); i++)
    {
        d = *i;
        for(int m = 0;m < sizeof(int);m++) {
            datas.push_back(p[m]);
        }
    }

    int data_size = datas.size();
    
    int size = data_size + sizeof(MapInfo);
    output->reserve(size + output->size() + sizeof(int));
    char * p_size = (char*)(& size);
    for(int i = 0;i < sizeof(int);i++) {
        output->push_back(p_size[i]);
    }
    char * p_info = (char*)(& info);
    for(int i = 0;i < sizeof(MapInfo);i++) {
        output->push_back(p_info[i]);
    }
    for(int i = 0;i < data_size;i++) {
        output->push_back(datas[i]);
    }
}
void LandmarkMap::from_char_array(char *buf, int size) {
    if(size < sizeof(MapInfo) + sizeof(int)) {
        return;
    }
    size = *((int*)buf) + sizeof(int);
    MapInfo * p_info = (MapInfo*)(buf + sizeof(int));
    info = *p_info;

    points.clear();
    has_init = true;
    if(size > sizeof(MapInfo) + sizeof(int)) {
        int *p_point = (int*)(buf + sizeof(int) + sizeof(MapInfo));
        int count = (size - sizeof(int) - sizeof(MapInfo)) / sizeof(int);
        for(int i = 0;i < count;i++) {
            points.insert(*p_point);
            p_point++;
        }
    }
}
//-----------------------------------------------------------------------------------

void LandmarkMapGenerator::add_point(double x, double y) {
    int map_key[2];
    map_key[0] = x / resolution;
    map_key[1] = y / resolution;
    long key = *((long*)map_key);
    
    auto search = grid_data.find(key);
    if(search == grid_data.end()) {
        grid_data.insert(std::pair<long, int>(key, 1));
        if(map_key[0] < x_min) {
            x_min = map_key[0];
        }
        if(map_key[1] < y_min) {
            y_min = map_key[1];
        }
        if(map_key[0] > x_max) {
            x_max = map_key[0];
        }
        if(map_key[1] > y_max) {
            y_max = map_key[1];
        }
    }
    else {
        grid_data[key] ++;
    }
}

LandmarkMap LandmarkMapGenerator::to_landmark_map() {
    MapInfo info;
    if(grid_data.empty()) {
        info.width = 0;
        info.height = 0;
        info.resolution = resolution;
        info.origen_x = 0.;
        info.origen_y = 0.;
        LandmarkMap ret(info);
        return ret;
    }

    info.width = x_max - x_min + 1;
    info.height = y_max - y_min + 1;
    info.origen_x = x_min * resolution;
    info.origen_y = y_min * resolution;
    info.resolution = resolution;
    float threshold_rate = get_configs()->get_float("map", "landmark_map_binarilize_threshold", nullptr);

    LandmarkMap ret(info);
    int count = 0;
    int key[2];
    long *p = (long*)key;
    for(auto &entry : grid_data) {
        *p = entry.first;
        int x = key[0];
        int y = key[1];
        int sum = 0;
        for(int m = x - 1; m <= x + 1; m++) {
            for(int n = y - 1; n <= y + 1; n++) {
                key[0] = m;
                key[1] = n;
                auto search = grid_data.find(*p);
                if(search != grid_data.end()) {
                    sum += search->second;
                }
            }
        }
        double threshold = (sum / 9.) * threshold_rate;
        if(entry.second > threshold) {
            ret.insert(info.map_xy_to_array_index(x - x_min, y - y_min));
            count++;
        }
    }

    printf("Generate landmark map with %d/%d point\n", count, grid_data.size());

    ret.remove_noise();
    return ret;
}

