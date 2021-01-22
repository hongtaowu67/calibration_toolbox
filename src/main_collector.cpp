// Main script to collect data

#include "handeye_data_collector.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "handeye_data_collect");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string save_dir = "/home/hongtao/Desktop/012121_panda_data_collect";
    std::vector< std::vector<double> > data_collection_config{
        std::vector<double> {1.69784,1.10977,-0.0404292,-0.826768,0.0011844,1.8602,0.754446},
        std::vector<double> {1.51137,1.10623,-0.0262181,-0.830024,0.00206027,1.8602,0.98279},
        std::vector<double> {1.38482,1.1174,-0.0495129,-0.825374,0.00204639,1.8602,1.0979},
        std::vector<double> {1.70137,1.11387,0.10682,-0.786402,0.00204181,1.86019,0.846663},
        std::vector<double> {1.7429,1.12304,0.14356,-0.786844,0.00204528,1.86062,0.732605},
        std::vector<double> {1.83074,1.11434,0.171841,-0.796531,0.00204848,1.86056,0.681627},
        std::vector<double> {1.79119,0.991712,0.14761,-0.799861,0.0022098,2.01109,0.739599},
        std::vector<double> {1.62925,1.03987,0.140795,-0.735777,0.00220214,2.01282,0.82748},
        std::vector<double> {1.52395,1.12066,0.0140735,-0.603632,-0.00205602,2.01699,0.882035},
        std::vector<double> {1.40015,1.03313,-0.00382565,-0.684821,-0.00134286,2.01617,1.00752},
        std::vector<double> {1.30952,1.00985,-0.0254526,-0.754786,-0.000769813,2.01618,1.08332},
        std::vector<double> {1.23373,0.710482,0.00288513,-1.25384,0.00098996,2.19316,1.06418},
        std::vector<double> {1.25393,0.750882,0.171826,-1.32534,0.00103421,2.25609,0.976822},
        std::vector<double> {1.33761,0.787551,0.244721,-1.30928,-0.0761768,2.29156,0.899672},
        std::vector<double> {1.53815,0.786164,0.264596,-1.31046,-0.0758681,2.29155,0.790454},
        std::vector<double> {1.69664,0.785848,0.251444,-1.30846,-0.0757783,2.29155,0.711733},
        std::vector<double> {1.67703,0.529004,0.263281,-1.48425,-0.0725099,2.4309,0.77724},
        std::vector<double> {1.66159,0.571245,0.0989212,-1.45018,-0.0725097,2.43566,0.776324},
        std::vector<double> {1.6498,0.62259,-0.0539778,-1.38769,-0.0701014,2.4357,0.964562},
        std::vector<double> {1.51556,0.620402,-0.0994198,-1.3881,-0.0700973,2.4357,1.06666}
    };


    std::string image_topic = "/camera/rgb/image_raw";

    HandeyeDataCollector Collector(save_dir, image_topic, data_collection_config);

    Collector.collectData();

    ROS_INFO("Exiting");
        
    return 0;
}