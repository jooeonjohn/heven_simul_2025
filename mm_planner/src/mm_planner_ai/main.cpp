#include "mm_planner_ai/planner_ai.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("~");
    planner_ai::Planner planner;
    planner.Init(nh);

    ros::spin();
}