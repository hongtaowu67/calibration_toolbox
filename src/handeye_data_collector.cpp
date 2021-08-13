// Class to collect data for handeye calibration
// Author: Hongtao Wu
// Institution: Johns Hopkins University
// Date: Jan 12, 2021

#include "handeye_data_collector.h"

HandeyeDataCollector::HandeyeDataCollector(
    const std::string& save_dir,
    const std::string& image_topic,
    const std::string& base_frame,
    const std::string& ee_frame,
    const std::vector< std::vector<double> >& data_collection_config
): _it(_nh)
{
    _save_dir = save_dir;
    _data_collection_config = data_collection_config;
    double vel        = 0.2;
    double acc        = 0.2;
    double sleep_time = 1.0;

    std::vector<double> home_config {0.0, -M_PI/4, 0.0, -2*M_PI/3, 0.0, M_PI/3, M_PI/4};
    
    _base_frame = base_frame; // Base frame     
    _ee_frame = ee_frame; // End effector frame
    
    // Set up robot
    _robot_ptr = new Panda (vel, 
                            acc, 
                            sleep_time, 
                            home_config, 
                            _ee_frame, 
                            _base_frame);

    // Set up image subscriber
    _img_sub = _it.subscribe(image_topic, 1, &HandeyeDataCollector::imageCb, this);
    ros::Duration(1.0).sleep();

    ROS_INFO("Finish setting up handeye data collector!");
}

// Save End Effector Xform
void HandeyeDataCollector::saveCurrentData(
    const std::string& pose_path,
    const std::string& img_path
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
    std::ofstream outfile(pose_path);

    // Write pose data
    if (!outfile)
        std::cout << "Cannot open " << pose_path << " to write data!!!";
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
    
    // Write image data
    cv::Mat image;
    cv::cvtColor(_cv_img_ptr->image, image, cv::COLOR_BRG2RGB);
    cv::imwrite(img_path, image);

    ROS_INFO("Finish saving pose results to %s", pose_path.c_str());
    ROS_INFO("Finish saving image results to %s", img_path.c_str());
}

// Collect data by going through the data config list
void HandeyeDataCollector::collectData()
{
    int config_num = _data_collection_config.size();
    
    geometry_msgs::TransformStamped curr_ee_pose;
    std::string xform_txt;
    std::string img_png;

    for (std::size_t i; i<config_num; i++)
    {
        _robot_ptr->moveToJoint(_data_collection_config[i]);
        ros::Duration(1.0).sleep();

        xform_txt = "/" + std::to_string(i) + "_robotpose.txt";
        img_png   = "/" + std::to_string(i) + "_img.png";
        saveCurrentData(_save_dir + xform_txt, _save_dir + img_png);
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("Finish collecting %d sets of data", config_num);
    _robot_ptr->moveToHome();
}

void HandeyeDataCollector::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        _cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}