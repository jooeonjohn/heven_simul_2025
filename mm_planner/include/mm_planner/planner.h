#ifndef MM_PLANNER_MM_PLANNER_PLANNER_H_
#define MM_PLANNER_MM_PLANNER_PLANNER_H_

#include <geometry_msgs/Pose2D.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/CollisionData.h>
#include <morai_msgs/CtrlCmd.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <mm_common/lane_info.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "mm_gps_tracker/gps_tracker.h"
#include "mm_dwa_tracker/mm_dwa_tracker.h"
#include "mm_cut_in/cut_in.h"
#include "mm_lane_tracker/lane_tracker.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <memory>


namespace planner
{
    class Planner
    {
    public:
        Planner();
        ~Planner();

        void Init(ros::NodeHandle &nh);
        void ReadMissionPoints(const std::string &filename);
        void MainPlanning();
        void ogmCallback(const nav_msgs::OccupancyGrid::ConstPtr &ogm_msg);
        void PoseCallback(const geometry_msgs::Pose2D &msg);
        void VelocityCallback(const morai_msgs::EgoVehicleStatus &msg);
        void CollisionCallback(const morai_msgs::CollisionData &msg);
        void laneCallback(const mm_common::lane_info &msg);
        void trafficCallback(const std_msgs::Int32 &msg);
        void PublishPathCallback(const ros::TimerEvent&);
        visualization_msgs::Marker PublishTargetMarker(geometry_msgs::PoseStamped target, std::string frame_id);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
        
    };
}

#endif // MM_PLANNER_MM_PLANNER_PLANNER_H_