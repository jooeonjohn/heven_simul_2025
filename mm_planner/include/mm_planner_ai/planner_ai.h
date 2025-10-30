#ifndef MM_PLANNER_MM_PLANNER_PLANNER_H_
#define MM_PLANNER_MM_PLANNER_PLANNER_H_

#include <geometry_msgs/Pose2D.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/CollisionData.h>
#include <morai_msgs/CtrlCmd.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "mm_gps_tracker/gps_tracker.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <memory>


namespace planner_ai
{
    class Planner
    {
    public:
        Planner();
        ~Planner();

        void Init(ros::NodeHandle &nh);
        void ReadMissionPoints(const std::string &filename);
        void MainPlanning();
        void PoseCallback(const morai_msgs::EgoVehicleStatus &msg);
        void CollisionCallback(const morai_msgs::CollisionData &msg);
        void trafficCallback(const std_msgs::Int32 &msg);
        void PublishPathCallback(const ros::TimerEvent&);
        visualization_msgs::Marker PublishTargetMarker(geometry_msgs::PoseStamped target, std::string frame_id);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
        
    };
}

#endif // MM_PLANNER_MM_PLANNER_PLANNER_H_