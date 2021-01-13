#ifndef HANDEYE_DATA_COLLECTOR_H
#define HANDEYE_DATA_COLLECTOR_H

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>

#include "calibration_toolbox/DataCollect.h"
#include "panda.h"
#include "utils.h"

class HandeyeDataCollector
{
public:
    HandeyeDataCollector(
        const std::string& save_dir,
        const std::vector< std::vector<double> >& data_collection_config
    );
    ~HandeyeDataCollector(){
        delete _robot_ptr;
    }

    void saveCurrentXform(const std::string& path);

private:
    // ROS handler
    ros::NodeHandle _nh;

    // Data directory to save the collected data
    std::string _save_dir;
    // Robot configuration for collecting data
    std::vector< std::vector<double> > _data_collection_config;

    // Robot
    Panda* _robot_ptr;
};

#endif // HANDEYE_DATA_COLLECTOR_H