// Main script to collect data for handeye calibration
// Author: Hongtao Wu
// Jan 12, 2021

#include <ros/ros.h>

#include "panda.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "handeye_data_collector");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    Panda Robot;

    return 0;
}