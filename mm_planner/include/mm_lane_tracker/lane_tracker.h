#ifndef MM_LANETRACKER_MM_LANETRACKER_LANETRACKER_H_
#define MM_LANETRACKER_MM_LANETRACKER_LANETRACKER_H_

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <morai_msgs/CtrlCmd.h>
#include <mm_common/lane_info.h>
#include <memory>
#include <string>


namespace lane_tracker
{
    class Lanetracker
    {
    public:
        Lanetracker();
        ~Lanetracker();

        void Init(float vel, float k, float kp, float ki, float kd);
        morai_msgs::CtrlCmd Stanley(mm_common::lane_info lane, float &curr_vel);
        float PID(float current_speed, float target_speed);


    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}


#endif // MM_LANETRACKER_MM_LANETRACKER_LANETRACKER_H_