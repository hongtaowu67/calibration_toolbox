// Class for handeye calibration data collector
// Author: Hongtao Wu
// Jan 13, 2020

#ifndef HANDEYE_DATA_COLLECTOR_H
#define HANDEYE_DATA_COLLECTOR_H

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "panda.h"
#include "utils.h"

class HandeyeDataCollector
{
public:
    HandeyeDataCollector(
        const std::string& save_dir,
        const std::string& image_topic,
        const std::string& base_frame,
        const std::string& ee_frame,
        const std::vector< std::vector<double> >& data_collection_config
    );
    
    ~HandeyeDataCollector(){
        delete _robot_ptr;
    }

    // Save the xform of the robot at the current instance
    void saveCurrentData(const std::string& pose_path, const std::string& img_path);

    // Main function to move the robot and collect data
    void collectData();

    // Image subscriber callback function
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

private:
    // ROS handler
    ros::NodeHandle _nh;

    // Data directory to save the collected data
    std::string _save_dir;

    // Base and end-effector frame
    std::string _base_frame;
    std::string _ee_frame;

    // Robot configuration for collecting data
    std::vector< std::vector<double> > _data_collection_config;

    // Robot
    Panda* _robot_ptr;

    // Image subscriber
    cv_bridge::CvImagePtr           _cv_img_ptr;
    image_transport::Subscriber     _img_sub;
    image_transport::ImageTransport _it;
};

#endif // HANDEYE_DATA_COLLECTOR_H