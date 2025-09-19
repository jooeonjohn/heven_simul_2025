#include "mm_pose_parser/pose_parser.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pose_parser");
    ros::NodeHandle nh("~");
    pose_parser::PoseParser parser;
    parser.Init(nh);

    ros::spin();
}