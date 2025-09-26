#include "mm_planner/planner.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("~");
    planner::Planner planner;
    planner.Init(nh);

    ros::spin();
}