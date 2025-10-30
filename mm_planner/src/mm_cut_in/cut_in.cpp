#include "mm_cut_in/cut_in.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

namespace cut_in
{
    struct CutIn::Impl
    {
        double origin_lat;
        double origin_lon;
        nav_msgs::Path path;
        float lad; // Lookahead distance in meters
        float vel;
        float k;
        morai_msgs::CtrlCmd cmd;
        morai_msgs::CtrlCmd stop_cmd;
        float kp; // pid
        float ki;
        float kd;
        float pid_integral;
        float pid_prev_error;
        ros::Time pid_prev_time;
        float resolution, center_offset;
        float occupancy_thresh;
        int cell_thresh;
    };

    CutIn::CutIn() : impl_(new Impl){}
    CutIn::~CutIn() {}

    void CutIn::Init(const nav_msgs::Path &path, float lad, float vel, float k, float kp, float ki, float kd, float occupancy_thresh, int cell_thresh)
    {
        impl_->path  = path;

        impl_->lad = lad;
        impl_->vel = vel;
        impl_->k = k;

        impl_->cmd.brake = 1.0;
        impl_->cmd.accel = 0.0;
        impl_->cmd.steering = 0.0;

        impl_->stop_cmd = impl_->cmd;

        impl_->kp = kp;
        impl_->ki = ki;
        impl_->kd = kd;

        impl_->resolution = 0.1;
        impl_->center_offset = 7.0/impl_->resolution;
        impl_->occupancy_thresh = occupancy_thresh;
        impl_->cell_thresh = cell_thresh;
    }

    geometry_msgs::PoseStamped CutIn::findTarget(const geometry_msgs::Pose2D &pose, size_t &target_index, float &curr_dist) {
        // --- Start by assuming the first waypoint is the closest ---
        size_t target_idx = 0;
        float dx = impl_->path.poses[0].pose.position.x - pose.x;
        float dy = impl_->path.poses[0].pose.position.y - pose.y;
        curr_dist = std::hypot(dx, dy);

        // --- Check the rest ---
        for (size_t i = 1; i < impl_->path.poses.size(); ++i)
        {
            dx = impl_->path.poses[i].pose.position.x - pose.x;
            dy = impl_->path.poses[i].pose.position.y - pose.y;
            float dist = std::hypot(dx, dy);

            if (dist < curr_dist)
            {
                curr_dist = dist;
                target_idx = i;
            }
        }

        while (target_idx < impl_->path.poses.size() && curr_dist < impl_->lad)
        {   
            if (target_idx+1 == impl_->path.poses.size()) target_idx = 0;
            else ++target_idx;

            dx = impl_->path.poses[target_idx].pose.position.x - pose.x;
            dy = impl_->path.poses[target_idx].pose.position.y - pose.y;
            curr_dist = std::hypot(dx, dy);
            
        }

        geometry_msgs::PoseStamped target = impl_->path.poses[target_idx];
        target_index = target_idx;
        return target;
    }

    morai_msgs::CtrlCmd CutIn::Stanley(const geometry_msgs::Pose2D &pose, const nav_msgs::OccupancyGrid::ConstPtr &ogm, geometry_msgs::PoseStamped &target, float &curr_vel, int current_mission)
    {
        float roi_x_min = 0.0;
        float roi_x_max = 7.0;
        float roi_y_min = -7.0;
        float roi_y_max = 7.0;
        
        if (current_mission == 1 || current_mission == 5){
            roi_y_min = -1.0;
        }

        float sum_occupancy = 0.0f;
        int cell_count = 0;

        for (float x = roi_x_min; x <= roi_x_max; x += impl_->resolution) {
            for (float y = roi_y_min; y <= roi_y_max; y += impl_->resolution) {
                int ogm_x = static_cast<int>((x + 1.0f) / impl_->resolution);
                int ogm_y = static_cast<int>(-(y / impl_->resolution) + impl_->center_offset);

                if (ogm_x >= 0 && ogm_x < ogm->info.width &&
                    ogm_y >= 0 && ogm_y < ogm->info.height) {

                    int index = ogm_y * ogm->info.width + ogm_x;
                    float occupancy = static_cast<float>(ogm->data[index]);

                    // Skip unknown cells (-1)
                    if (occupancy >= 0) {
                        sum_occupancy += occupancy;
                        ++cell_count;
                    }
                }
            }
        }

        float ogm_mean = (cell_count > 0) ? (sum_occupancy / static_cast<float>(cell_count)) : 0.0f;

        std::cout << "OGM mean: " << ogm_mean
                << " (thresh=" << impl_->occupancy_thresh << ")" << std::endl;
        std::cout << "OGM cell count: " << cell_count
                << " (thresh=" << impl_->cell_thresh << ")" << std::endl;

        if (ogm_mean > impl_->occupancy_thresh, cell_count > impl_->cell_thresh)
            return impl_->stop_cmd;

        // --- Start by assuming the first waypoint is the closest ---
        size_t target_index = 0;
        float curr_dist = 0.0;
        target = CutIn::findTarget(pose, target_index, curr_dist);

        float path_theta = 0.0f;

        if (target_index+1 < impl_->path.poses.size()) {
            auto &curr = impl_->path.poses[target_index].pose.position;
            auto &next = impl_->path.poses[target_index + 1].pose.position;
            path_theta = std::atan2(next.y - curr.y, next.x - curr.x);
        } 
        else {
            auto &curr = impl_->path.poses[target_index].pose.position;
            auto &prev = impl_->path.poses[target_index - 1].pose.position;
            path_theta = std::atan2(curr.y - prev.y, curr.x - prev.x);
        }
        // Calculate the heading error
        float heading_error = path_theta - pose.theta * M_PI / 180.0;
        // Normalize the heading error to [-pi, pi]
        while (heading_error > M_PI) heading_error -= 2 * M_PI;
        while (heading_error < -M_PI) heading_error += 2 * M_PI;
        // Calculate the cross-track error
        float target_theta = std::atan2(target.pose.position.y - pose.y, target.pose.position.x - pose.x) - path_theta;
        float cross_track_error = std::hypot(target.pose.position.x - pose.x, target.pose.position.y - pose.y) * std::sin(target_theta);
        // Calculate the steering angle using the Stanley controller formula
        float steering_angle = (heading_error + std::atan2(impl_->k * cross_track_error, impl_->vel/3.6)); // use m/s

        // Set the command values
        impl_->cmd.accel = std::max(std::min(PID(curr_vel, impl_->vel), 1.0f), 0.0f);
        impl_->cmd.steering = steering_angle/(40*M_PI/180);
        impl_->cmd.brake = 0;


        // std::cout << "Nearest waypoint index: " << target_index 
        //         << " at (" << target.pose.position.x << ", " << target.pose.position.y 
        //         << "), distance = " << curr_dist << std::endl;

        // std::cout << "Current pose: (" << pose.x << ", " << pose.y << ", " << pose.theta
        //         << ")" << std::endl;

        std::cout << "Heading error: " << heading_error * 180.0 / M_PI
                  << ", Cross-track error: " << cross_track_error
                  << std::endl;

        std::cout << "Steering angle: " << steering_angle << std::endl;
        return impl_->cmd;
    }

    float CutIn::PID(float current_speed, float target_speed)
    {
        ros::Time now = ros::Time::now();
        float error = target_speed - current_speed;

        if (impl_->pid_prev_time.isZero()) {
            // First call: use only proportional control
            impl_->pid_prev_error = error;
            impl_->pid_prev_time = now;
            impl_->pid_integral = 0.0;
            return impl_->kp * error;
        }

        float dt = (now - impl_->pid_prev_time).toSec();
        impl_->pid_prev_time = now;

        impl_->pid_integral += error * dt;
        float integral_max = target_speed * 10;
        impl_->pid_integral = std::max(std::min(impl_->pid_integral, integral_max), -integral_max);
        float derivative = (dt > 0.0) ? (error - impl_->pid_prev_error) / dt : 0.0;
        impl_->pid_prev_error = error;

        float output = impl_->kp * error + impl_->ki * impl_->pid_integral + impl_->kd * derivative;

        // std::cout << "error: " << error << ", throttle: " << output << std::endl;
        return output;
    }
}
