// Main script to collect data

#include "handeye_data_collector.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "handeye_data_collect");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string save_dir = "/home/hongtao/Desktop/011921_panda";
    std::vector< std::vector<double> > data_collection_config{
        std::vector<double> {0.317405,-1.01347,1.3687,-2.07375,0.621795,1.6796,0.808695},
        std::vector<double> {0.503307,-0.526671,1.16741,-2.56615,0.409771,2.37709,0.496006},
        std::vector<double> {0.942247,-0.419359,0.508364,-2.49337,0.418889,2.15797,0.508762},
        std::vector<double> {0.943158,-0.0215992,0.312029,-2.35252,0.417231,2.15745,0.496644},
        std::vector<double> {0.106008,-0.0263641,0.896944,-2.32976,0.461899,2.07419,0.0670142},
        std::vector<double> {0.629911,-0.919042,0.582827,-2.83403,0.735654,2.01993,-0.012623},
        std::vector<double> {0.0566064,-0.636599,1.30619,-2.29551,0.669226,2.01422,0.225199},
        std::vector<double> {1.74772,0.417972,-0.313529,-1.61076,0.197575,1.64264,0.670254},
        std::vector<double> {1.54043,-0.390261,-0.734448,-2.6848,0.296748,2.31246,0.287019},
        std::vector<double> {1.3873,-0.614913,0.0870832,-2.79928,0.285107,2.31083,0.602792},
        std::vector<double> {1.13804,-0.814492,0.816755,-2.84713,0.291491,2.46715,0.786067},
        std::vector<double> {1.13167,-0.31376,0.363799,-2.49977,0.28077,2.22852,0.588351},
        std::vector<double> {1.13041,-0.146241,0.37074,-2.3812,0.00771661,2.25023,-0.805397},
        std::vector<double> {1.15733,0.420697,0.316561,-1.62292,-0.0910725,1.72711,-0.685474},
        std::vector<double> {0.864443,0.398115,0.312225,-1.69749,0.256215,1.72616,-1.35603},
        std::vector<double> {1.09845,-0.133272,0.597986,-2.32405,-0.241746,2.12741,-0.43986}
    };

    std::string image_topic = "/camera/rgb/image_raw";

    HandeyeDataCollector Collector(save_dir, image_topic, data_collection_config);

    Collector.collectData();

    ROS_INFO("Exiting");
        
    return 0;
}