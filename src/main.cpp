// Main script to collect data

#include "handeye_data_collector.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "handeye_data_collect");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string save_dir = "/home/hongtao/Desktop";
    std::vector< std::vector<double> > data_collection_config;

    std::vector<double> config1 {0.0, -M_PI/4, 0.0, -2*M_PI/3, 0.0, M_PI/3, M_PI/4};
    data_collection_config.push_back(config1);
    
    config1[0] = M_PI/5;
    data_collection_config.push_back(config1);
    
    config1[0] = 0;
    config1[1] = -M_PI/3;
    data_collection_config.push_back(config1);

    config1[0] = -M_PI/5;
    data_collection_config.push_back(config1);

    std::string image_topic = "/camera/rgb/image_raw";

    HandeyeDataCollector Collector(save_dir, image_topic, data_collection_config);

    Collector.collectData();

    ROS_INFO("Exiting");
        
    return 0;
}