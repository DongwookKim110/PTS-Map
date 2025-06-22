#include "PTS_Map/PTS_Map.hpp"
#include <pcl/console/print.h>

int main(int argc, char** argv){
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    ros::init(argc, argv, "PTS_MAP");

    PTS_MAP pts_map;
    pts_map.run();

    ros::spin();

    return 0;
}