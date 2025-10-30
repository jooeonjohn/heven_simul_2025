#ifndef MM_CUTIN_MM_CUTIN_CUTIN_H_
#define MM_CUTIN_MM_CUTIN_CUTIN_H_

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <morai_msgs/CtrlCmd.h>
#include <memory>
#include <string>


namespace cut_in
{
    class CutIn
    {
    public:
        CutIn();
        ~CutIn();

        void Init(const nav_msgs::Path &path, float lad, float vel, float k, float kp, float ki, float kd, float ogm_thresh, int cell_thresh);
        geometry_msgs::PoseStamped findTarget(const geometry_msgs::Pose2D &pose, size_t &target_index, float &curr_dist);
        morai_msgs::CtrlCmd Stanley(const geometry_msgs::Pose2D &pose, const nav_msgs::OccupancyGrid::ConstPtr &ogm, geometry_msgs::PoseStamped &target,float &curr_vel, int current_mission);
        float PID(float current_speed, float target_speed);


    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}


#endif // MM_CUTIN_MM_CUTIN_CUTIN_H_