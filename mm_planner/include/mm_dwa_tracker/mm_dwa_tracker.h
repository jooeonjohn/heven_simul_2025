#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h> 
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <morai_msgs/CtrlCmd.h>
#include <memory>
#include <iostream>
#include <cmath>
#include <string>

namespace dwa_tracker {

    class dwaTracker {
    public:
        dwaTracker();
        ~dwaTracker();

        struct PathOutput {
            nav_msgs::Path path;
            float score;
        };

        // Core methods
        void Init(const std::string& pathfile, float lad, float max_speed, float min_speed, float max_omega, 
                   float min_omega, float dt, float window, float goal_weight, float obs_weight, int filter_window_size);
        geometry_msgs::PoseStamped transformationToBaseLink(const geometry_msgs::PoseStamped &target_map_frame);
        morai_msgs::CtrlCmd dynamicWindowApproach(geometry_msgs::Pose2D &pose, 
            const nav_msgs::OccupancyGrid::ConstPtr &ogm, nav_msgs::Path &dwa_path, geometry_msgs::PoseStamped &target);
        void applyMovingAverageFilter(float& speed, float& steer);
        PathOutput simulateTrajectory(float v, float theta);
        float isCollision(float x, float y);
        float evaluateTrajectory(float x, float y, float theta, float v);

    private:    
    
        tf2_ros::Buffer tfBuffer_;
        std::unique_ptr<tf2_ros::TransformListener> tfListener_;
        std::deque<float> speed_history;
        std::deque<float> steer_history;
        const size_t filter_window_size = 10;

        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}

#endif // DWA_TRACKER_H
