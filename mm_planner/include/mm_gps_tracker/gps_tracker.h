#ifndef MM_GPSTRACKER_MM_GPSTRACKER_GPSTRACKER_H_
#define MM_GPSTRACKER_MM_GPSTRACKER_GPSTRACKER_H_

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <morai_msgs/CtrlCmd.h>
#include <memory>
#include <string>


namespace gps_tracker
{
    class GPStracker
    {
    public:
        GPStracker();
        ~GPStracker();

        nav_msgs::Path Init(const std::string &pathfile, float lad, float vel, float k, float kp, float ki, float kd);
        void ReadCSV(const std::string &filename);
        geometry_msgs::PoseStamped findTarget(const geometry_msgs::Pose2D &pose, size_t &target_index, float &curr_dist);
        morai_msgs::CtrlCmd Stanley(const geometry_msgs::Pose2D &pose, geometry_msgs::PoseStamped &target,float &curr_vel, bool is_slow);
        float PID(float current_speed, float target_speed);


    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}


#endif // MM_GPSTRACKER_MM_GPSTRACKER_GPSTRACKER_H_