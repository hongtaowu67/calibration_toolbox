// Main script to collect data

#include "handeye_data_collector.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "handeye_data_collect");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string save_dir = "/home/hongtao/Desktop";
    std::vector< std::vector<double> > data_collection_config;

    HandeyeDataCollector Collector(save_dir, data_collection_config);
    
    return 0;
}