#ifndef CARTO_MODULE_H
#define CARTO_MODULE_H

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <list>

#include <Eigen/Core>

#include <cartographer/common/fixed_ratio_sampler.h>
#include <cartographer/common/time.h>

#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/pose_extrapolator.h>


#include "common/pose.h"
#include "common/simple_grid_map.h"
#include "common/landmark_map.h"
#include "msg_queue.h"

using namespace cartographer;

constexpr double radar_scan_time = 0.0001;

class CartoModuleOption {
public:
    CartoModuleOption(char *dir, char *file):config_files_dir(dir), config_file_name(file){};
    std::string     config_files_dir;
    std::string     config_file_name;
    std::string     range_name = "range0";
    std::string     imu_name = "imu0";
    std::string     odom_name = "odom0";
    double          radar_scan_time = 0.0001;
    double          pose_mathching_score = 0.95;
    int             paint_map_period_s = 1;
    MsgQueue *      msg_queue;
    int             pose_channel;
    int             imu_channel;
    int             radar_channel;
};

class CartoModule : public MsgQueueListener {
public:
    CartoModule(CartoModuleOption &opt);
    ~CartoModule();

    int handle_radar_data(PointCloudData & data);
    int handle_imu_data(ImuData2D & data);
    int handle_odom_data(Position & position);

    void paint_map(std::vector<char> *output);

    void stop_and_optimize();

    virtual void recieve_message(const int channel, char *buf, const int size);

private:
    CartoModuleOption option;
    pthread_mutex_t sensor_mutex;
    long latest_sensor_timestamp = -1;

    int imu_listener_id;
    int radar_listener_id;
    int odom_listener_id;

    pthread_mutex_t paint_mutex;

    std::unique_ptr<mapping::MapBuilderInterface> map_builder; //建图接口MapBuilder
    mapping::TrajectoryBuilderInterface *trajectory_builder; //路径接口指针TrajectoryBuilder
    mapping::proto::MapBuilderOptions map_builder_options; //MapBuilder参数
    mapping::proto::TrajectoryBuilderOptions trajectory_builder_options; //TrajectoryBuilder参数
    int trajectory_id;

    void OnLocalSlamResult(
        const int trajectory_id, const common::Time time,
        const transform::Rigid3d local_pose,
        sensor::RangeData range_data_in_local,
        const std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult> insertion_result);
    void OnLocalSlamResult2(
        const int trajectory_id, const common::Time time,
        const transform::Rigid3d local_pose,
        sensor::RangeData range_data_in_local);

    std::map<long, std::vector<Eigen::Vector2d>*> landmarks;
    double intensity_threshold = 0.8;
    LandmarkMapGenerator landmark_map;

    double landmark_rate = 0.1;
	
	Position radar_position;

	Position odom_position;
    int odom_count = 0;
    int skip_odometer_data = 0;
};

#endif
