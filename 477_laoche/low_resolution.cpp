
#include <cmath>
#include <stdio.h>

#include "low_resolution.h"
Low_resolution::Low_resolution(SimpleGridMap *map,double  thres)
{
	
	map_info = map->get_info();
	datas = map->datas;
	landmark = map->landmark;
	cap_points = map->cap_points;
	threshold = thres;
}


SimpleGridMap* Low_resolution::low_reso_map()
{
	if (datas.size() <= 0) return NULL;
	std::vector<char> re_datas;
	int w = map_info.width/2;
	int h = map_info.height/2;
	printf("原始地图的高%d宽%d\n",map_info.height,map_info.width);
	float total_value;
	int dsize = w  * h;
	re_datas.reserve(dsize);
	for(int m=0;m<dsize;m++)
	re_datas.push_back(255);
	for (int m = 0; m<h; ++m)//计算降质后图像大小
	{	for (int n = 0; n < w; ++n)
		{
			//取原图上2*2方阵的像素据均值给降质图像的每个像素
			total_value = 0;
			for (int xi = 2 * m; xi < (2 * m + 2); xi++)
			{
				for (int yi = 2 * n; yi < (2 * n + 2); yi++)
				{
					total_value += datas[map_info.map_xy_to_array_index(yi, xi)];
					//if(total_value < datas[map_info.map_xy_to_array_index(yi, xi)])
					//total_value = datas[map_info.map_xy_to_array_index(yi, xi)];
				}
			}
			re_datas[n + m*w] = total_value/4;
			//printf(" %f ",total_value/4.0);
			//if (total_value < (0.3 * 400))
			//	{re_datas[n + m*w] = 99;
				//printf("output 9999999\n");
			//	}
			//else
			//	{re_datas[n + m*w] = 0;
			//	//printf("output 000000\n");
			//	}
			//取原图上2*2方阵的像素赋值给降质图像的每个像素
		}
		//printf("\n");
	}
	MapInfo newmap_info;
	newmap_info.width = w;
	newmap_info.height = h;
	newmap_info.origen_x = map_info.origen_x;
	newmap_info.origen_y = map_info.origen_y;
	newmap_info.resolution = map_info.resolution*2;
	printf("修改后地图的高%d宽%d\n",newmap_info.height,newmap_info.width);
	gridmap_ret = new SimpleGridMap(newmap_info);
	gridmap_ret->datas = re_datas;
	printf("re_datas size=%d,gridmap_ret->datas.size=%d\n",re_datas.size(),gridmap_ret->datas.size());
	gridmap_ret->landmark = landmark;
	gridmap_ret->cap_points = cap_points;
	return (gridmap_ret);
}
double Low_resolution::get(double x, double y) {
    int x_map = map_info.global_x_to_map_x(x);
    int y_map = map_info.global_y_to_map_y(y);
    if(x_map < 0 || y_map < 0 || x_map >= map_info.width || y_map >= map_info.height) {
        return -1;
    }
    return datas[map_info.map_xy_to_array_index(x_map, y_map)];
}
