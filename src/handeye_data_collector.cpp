// Class to collect data for handeye calibration
// Author: Hongtao Wu
// Jan 12, 2021

#include "handeye_data_collector.h"


HandeyeDataCollector::HandeyeDataCollector(
    const std::string& save_dir,
    const std::vector< std::vector<double> >& data_collection_config
)
{
    _save_dir = save_dir;
    _data_collection_config = data_collection_config;
    double vel        = 0.5;
    double acc        = 0.5;
    double sleep_time = 1.0;

    std::vector<double> home_config {0.0, -M_PI/4, 0.0, -2*M_PI/3, 0.0, M_PI/3, M_PI/4};

    std::string ee_frame = "panda_EE"; // End effector frame
    std::string base_frame = "panda_link0"; // Base frame 

    _robot_ptr = new Panda (vel, 
                            acc, 
                            sleep_time, 
                            home_config, 
                            ee_frame, 
                            base_frame);

    ROS_INFO("Testing saving xform");
    saveCurrentXform(_save_dir + "/franka_ee_test.txt");
    ROS_INFO("Finish setting up handeye data collector!");
}

// Save End Effector Xform
void HandeyeDataCollector::saveCurrentXform(
    const std::string& path
)
{
    geometry_msgs::TransformStamped ee_xform = _robot_ptr->getCurrentEEPose();
    double x, y, z;
    double q_w, q_x, q_y, q_z;

    x = ee_xform.transform.translation.x;
    y = ee_xform.transform.translation.y;
    z = ee_xform.transform.translation.z;

    q_w = ee_xform.transform.rotation.w;
    q_x = ee_xform.transform.rotation.x;
    q_y = ee_xform.transform.rotation.y;
    q_z = ee_xform.transform.rotation.z;

    Eigen::Matrix3d R = quat2rotm(q_w, q_x, q_y, q_z);
    std::ofstream outfile(path);

    if (!outfile)
        std::cout << "Cannot open " << path << " to write data!!!";
    else
        outfile << R(0, 0) << " "
                << R(0, 1) << " "
                << R(0, 2) << " "
                << x << " "
                << R(1, 0) << " "
                << R(1, 1) << " "
                << R(1, 2) << " "
                << y << " "
                << R(2, 0) << " "
                << R(2, 1) << " "
                << R(2, 2) << " "
                << z << " "
                << 0 << " "
                << 0 << " "
                << 0 << " "
                << 1 << " ";
    
    ROS_INFO("Finish saving results to %s", path.c_str());
}