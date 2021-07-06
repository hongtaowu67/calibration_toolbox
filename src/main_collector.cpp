// Main script to collect data

#include "handeye_data_collector.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "handeye_data_collect");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string save_dir = "/home/hongtao/Desktop/041321_panda_ps";

    // Calibration waypoints for PrimeSense (no gripper)
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
    
    // // Calibration waypoints for PrimeSense (with gripper)
    // // Apr 10, 2021
    // std::vector< std::vector<double> > data_collection_config{
    //     std::vector<double> {1.3033,0.885872,-0.00101988,-1.15998,0.0109521,2.26731,1.15363},
    //     std::vector<double> {1.40318,0.87811,0.0139547,-1.16887,0.0103098,2.2673,1.11995},
    //     std::vector<double> {1.45089,0.878285,0.0653002,-1.20711,0.0103002,2.2673,1.07318},
    //     std::vector<double> {1.58707,0.868433,0.0672119,-1.21931,-0.00754694,2.2673,0.936864},
    //     std::vector<double> {1.71013,0.904899,0.0305817,-1.14166,-0.10296,2.26941,0.874658},
    //     std::vector<double> {1.82074,0.896208,0.0433646,-1.11249,-0.085683,2.26935,0.766914},
    //     std::vector<double> {1.89702,0.922749,0.0389887,-1.15587,-0.0857948,2.26933,0.698874},
    //     std::vector<double> {1.95098,0.926347,0.0365203,-1.15705,-0.0861039,2.26932,0.644494},
    //     std::vector<double> {1.94872,0.846391,0.0375736,-1.19345,-0.0860771,2.39761,0.675752},
    //     std::vector<double> {1.90039,0.813704,-0.00301725,-1.24852,-0.0860646,2.39764,0.695702},
    //     std::vector<double> {1.85007,0.726207,-0.0415209,-1.33843,-0.0861973,2.56578,0.843243},
    //     std::vector<double> {1.78374,0.672009,-0.0792318,-1.36957,-0.0861126,2.56613,0.942345},
    //     std::vector<double> {1.773,0.613048,-0.199562,-1.40513,-0.086181,2.55771,1.01138},
    //     std::vector<double> {1.78129,0.593449,-0.292387,-1.51991,-0.0858169,2.59828,1.12689},
    //     std::vector<double> {1.70928,0.555495,-0.28844,-1.59547,-0.086174,2.59839,1.20856},
    //     std::vector<double> {1.67411,0.532852,-0.300983,-1.64358,-0.0861852,2.5985,1.30186},
    //     std::vector<double> {1.64651,0.375668,-0.303108,-1.81617,-0.0861868,2.77683,1.33388},
    //     std::vector<double> {1.7281,0.354263,-0.218029,-1.77539,-0.086905,2.82266,1.20092},
    //     std::vector<double> {1.78441,0.350948,-0.202494,-1.78312,-0.0887584,2.83999,1.08953},
    //     std::vector<double> {1.87671,0.377335,-0.128321,-1.72165,-0.088777,2.8399,0.902002},
    //     std::vector<double> {1.96362,0.363944,-0.131839,-1.73238,-0.0885221,2.87841,0.817462}
    // };

    // Calibration waypoints for RealSense (with gripper)
    // Apr 10, 2021
    // std::vector< std::vector<double> > data_collection_config{
    //     std::vector<double> {1.57939,0.624905,-0.217326,-1.51415,0.281146,1.43553,0.428142},
    //     std::vector<double> {1.53054,0.428986,-0.20305,-1.78846,0.292658,1.58943,0.365625},
    //     std::vector<double> {1.37868,-0.141957,-0.221902,-2.55709,0.441619,2.15381,0.234671},
    //     std::vector<double> {1.43711,-0.407843,-0.404061,-2.77728,0.441914,2.22775,0.103082},
    //     std::vector<double> {1.61998,-0.493948,-0.707571,-2.98228,0.474855,2.50507,-0.0917585},
    //     std::vector<double> {1.39258,-0.66346,0.196858,-3.06303,0.472124,2.5883,0.391077},
    //     std::vector<double> {1.11469,-0.523576,0.488524,-2.83761,0.478909,2.38422,0.415195},
    //     std::vector<double> {0.312337,-0.431629,1.26141,-2.40736,0.480388,2.02592,0.394964},
    //     std::vector<double> {-0.26087,-0.527319,1.76309,-2.07033,0.481091,1.7631,0.286615},
    //     std::vector<double> {-0.531032,-0.577864,1.97122,-1.8272,0.480077,1.58615,0.463581},
    //     std::vector<double> {-0.697791,-0.682944,2.09453,-1.66969,0.480734,1.45241,0.47287},
    //     std::vector<double> {-0.558019,-0.8552,2.04616,-1.50754,0.471862,1.34958,0.643449},
    //     std::vector<double> {-0.40557,-0.754453,1.93381,-1.67583,0.471166,1.46693,0.656287},
    //     std::vector<double> {-0.17519,-0.635321,1.75802,-1.91312,0.465168,1.61659,0.634519},
    //     std::vector<double> {0.145206,-0.568122,1.50468,-2.11056,0.468949,1.75089,0.672345},
    //     std::vector<double> {0.48055,-0.617305,1.25977,-2.31805,0.468924,1.88793,0.675008},
    //     std::vector<double> {0.899939,-0.800963,1.01227,-2.60099,0.479067,2.0712,0.600476},
    //     std::vector<double> {1.23268,-0.986042,0.787681,-2.82064,0.479354,2.16179,0.686628},
    //     std::vector<double> {1.2398,-1.11876,1.07492,-2.76517,0.525215,2.08784,0.731314},
    //     std::vector<double> {0.358488,-1.10681,1.62533,-2.09523,0.579721,1.74578,0.667733},
    //     std::vector<double> {-0.355703,-1.06549,1.8299,-1.48011,0.596011,1.26494,0.644632}
    // };

    std::string image_topic = "/camera/rgb/image_raw";
    
    std::string base_frame = "panda_link0";
    std::string ee_frame = "panda_link8";

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