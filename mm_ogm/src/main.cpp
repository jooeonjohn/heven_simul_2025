#include "mm_ogm.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mm_ogm");
    ros::NodeHandle nh("~");
    local_ogm_builder::LocalOGMBuilder builder(nh);
    ros::spin();
    return 0;
}
