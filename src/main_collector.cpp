// Main script to collect data

#include "handeye_data_collector.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "handeye_data_collect");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string save_dir = "/home/hongtao/Desktop/041021_panda_data_collect";
    
    std::vector< std::vector<double> > data_collection_config{
        std::vector<double> {1.3033,0.885872,-0.00101988,-1.15998,0.0109521,2.26731,1.15363},
        std::vector<double> {1.40318,0.87811,0.0139547,-1.16887,0.0103098,2.2673,1.11995},
        std::vector<double> {1.45089,0.878285,0.0653002,-1.20711,0.0103002,2.2673,1.07318},
        std::vector<double> {1.58707,0.868433,0.0672119,-1.21931,-0.00754694,2.2673,0.936864},
        std::vector<double> {1.71013,0.904899,0.0305817,-1.14166,-0.10296,2.26941,0.874658},
        std::vector<double> {1.82074,0.896208,0.0433646,-1.11249,-0.085683,2.26935,0.766914},
        std::vector<double> {1.89702,0.922749,0.0389887,-1.15587,-0.0857948,2.26933,0.698874},
        std::vector<double> {1.95098,0.926347,0.0365203,-1.15705,-0.0861039,2.26932,0.644494},
        std::vector<double> {1.94872,0.846391,0.0375736,-1.19345,-0.0860771,2.39761,0.675752},
        std::vector<double> {1.90039,0.813704,-0.00301725,-1.24852,-0.0860646,2.39764,0.695702},
        std::vector<double> {1.85007,0.726207,-0.0415209,-1.33843,-0.0861973,2.56578,0.843243},
        std::vector<double> {1.78374,0.672009,-0.0792318,-1.36957,-0.0861126,2.56613,0.942345},
        std::vector<double> {1.773,0.613048,-0.199562,-1.40513,-0.086181,2.55771,1.01138},
        std::vector<double> {1.78129,0.593449,-0.292387,-1.51991,-0.0858169,2.59828,1.12689},
        std::vector<double> {1.70928,0.555495,-0.28844,-1.59547,-0.086174,2.59839,1.20856},
        std::vector<double> {1.67411,0.532852,-0.300983,-1.64358,-0.0861852,2.5985,1.30186},
        std::vector<double> {1.64651,0.375668,-0.303108,-1.81617,-0.0861868,2.77683,1.33388},
        std::vector<double> {1.7281,0.354263,-0.218029,-1.77539,-0.086905,2.82266,1.20092},
        std::vector<double> {1.78441,0.350948,-0.202494,-1.78312,-0.0887584,2.83999,1.08953},
        std::vector<double> {1.87671,0.377335,-0.128321,-1.72165,-0.088777,2.8399,0.902002},
        std::vector<double> {1.96362,0.363944,-0.131839,-1.73238,-0.0885221,2.87841,0.817462}
    };


    std::string image_topic = "/camera/rgb/image_raw";
    
    std::string base_frame = "panda_link0";
    std::string ee_frame = "panda_hand";

    HandeyeDataCollector Collector(
        save_dir, 
        image_topic, 
        base_frame, 
        ee_frame, 
        data_collection_config
    );

    Collector.collectData();

    ROS_INFO("Exiting");
        
    return 0;
}