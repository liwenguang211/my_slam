#include "simple_grid_map.h"
#include "huffman.h"

#include <stdio.h>

SimpleGridMap::SimpleGridMap(MapInfo &info) : map_info(info)
{
    pthread_mutex_init(&mutex, nullptr);
};

SimpleGridMap::SimpleGridMap(SimpleGridMap &map)
{
    pthread_mutex_init(&mutex, nullptr);
    map_info = map.get_info();
    datas = map.datas;
    landmark = map.landmark;
    cap_points = map.cap_points;
}

SimpleGridMap::SimpleGridMap(char *data_buf, int buf_size)
{
    pthread_mutex_init(&mutex, nullptr);
    if(buf_size == 0) {
        return;
    }
    from_char_array(data_buf, buf_size);
};

SimpleGridMap::~SimpleGridMap()
{
    pthread_mutex_destroy(&mutex);
};

void SimpleGridMap::to_char_array(std::vector<char> *output) {
    pthread_mutex_lock(&mutex);
    std::vector<char> compressed;
    int data_size = 0;
    if (datas.size() != 0)
    {
        data_size = huffman_compress(&datas, &compressed);
    }
    int size = data_size + sizeof(MapInfo);
    output->reserve(output->size() + size + sizeof(int));
    char *p_size = (char *)(&size);
    for (int i = 0; i < sizeof(int); i++)
    {
        output->push_back(p_size[i]);
    }
    char *p_info = (char *)(&map_info);
    for (int i = 0; i < sizeof(MapInfo); i++)
    {
        output->push_back(p_info[i]);
    }
    for (int i = 0; i < data_size; i++)
    {
        output->push_back(compressed[i]);
    }
    pthread_mutex_unlock(&mutex);

    landmark.to_char_array(output);

    for(CapturePoint &point : cap_points) {
        point.to_char_array(output);
    }
}

void SimpleGridMap::from_char_array(char *buf, int size) {
    if (size < sizeof(int))
    {
        printf("Decompress map data error : No map data\n");
        return;
    }
    int char_size = *((int *)buf);
    if (size < char_size + sizeof(int))
    {
        printf("Decompress map data error : Incomplete data\n");
        return;
    }
    MapInfo info = *((MapInfo *)(buf + sizeof(int)));
    map_info = info;
    int min = 0, max = 0;
    int sum = 0;
    datas.reserve(info.width * info.height);
    std::vector<char> compressed;
    for (int i = sizeof(int) + sizeof(MapInfo); i < char_size + sizeof(int); i++)
    {
        compressed.push_back(buf[i]);
    }
    int ret = huffman_decompress(&compressed, &datas);
    if (ret < info.width * info.height)
    {
        printf("Decompress map data error\n");
        datas.clear();
        datas.assign(info.width * info.height, -1);
        return;
    }
    printf("New map infomation: %dx%d, origin (%.3f, %.3f), resolution %f \n", info.width, info.height, info.origen_x, info.origen_y, info.resolution);

    int pos = char_size + sizeof(int);
    char_size = *((int *)(buf + pos));
    landmark.from_char_array(buf + pos, char_size);

    pos += char_size + sizeof(int);
    
    cap_points.clear();
    while(pos < size) {
        char_size = *((int *)(buf + pos));
        CapturePoint cap(buf + pos, char_size);
        pos += char_size + sizeof(int);
    }
    printf("Read %d capture points\n", cap_points.size());

}

double SimpleGridMap::get(double x, double y)
{
    pthread_mutex_lock(&mutex);
    int x_map = map_info.global_x_to_map_x(x);
    int y_map = map_info.global_y_to_map_y(y);
    if (x_map < 0 || y_map < 0 || x_map >= map_info.width || y_map >= map_info.height)
    {
        pthread_mutex_unlock(&mutex);
        return -1;
    }
    int data = (datas[map_info.map_xy_to_array_index(x_map, y_map)] & 0xff);
    pthread_mutex_unlock(&mutex);
    if (data == 0xff)
    {
        return -1;
    }
    return ((double)data / 100.);
}

void SimpleGridMap::binarization(double threshold) {
    int thre = (int)(threshold * 100);
    pthread_mutex_lock(&mutex);
    int size = map_info.height * map_info.width;
    for(int i = 0;i < size;i++) {
        int v = datas[i] & 0xff;
        if(v == 0xff) {
            continue;
        }
        else if(v < thre) {
            datas[i] = 0;
        }
        else {
            datas[i] = 99;
        }
    }
    pthread_mutex_unlock(&mutex);
}
