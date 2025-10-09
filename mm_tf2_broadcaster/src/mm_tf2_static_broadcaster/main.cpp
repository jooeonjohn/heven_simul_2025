#include "mm_tf2_static_broadcaster/tf2_static_broadcaster.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mm_tf2_static_broadcaster");
    ros::NodeHandle nh("~");
    mm_tf2_static_broadcaster::TF2StaticBroadcaster tf2sb;
    tf2sb.Init(nh);
    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        tf2sb.SpinOnce();
        rate.sleep();
    }
}
