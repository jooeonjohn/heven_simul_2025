#include "mm_gps_tracker/gps_tracker.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

namespace gps_tracker
{
    struct GPStracker::Impl
    {
        double origin_lat;
        double origin_lon;
        nav_msgs::Path path;
        float lad; // Lookahead distance in meters
        float vel;
        float k;
        morai_msgs::CtrlCmd cmd;
        float kp; // pid
        float ki;
        float kd;
        float pid_integral;
        float pid_prev_error;
        ros::Time pid_prev_time;
    };

    GPStracker::GPStracker() : impl_(new Impl){}
    GPStracker::~GPStracker() {}

    nav_msgs::Path GPStracker::Init(const std::string &pathfile, float lad, float vel, float k, float kp, float ki, float kd)
    {
        ReadCSV(pathfile);
        impl_->lad = lad;
        impl_->vel = vel;
        impl_->k = k;

        impl_->cmd.brake = 1.0;
        impl_->cmd.accel = 0.0;
        impl_->cmd.steering = 0.0;

        impl_->kp = kp;
        impl_->ki = ki;
        impl_->kd = kd;
        return impl_->path;
    }

    void GPStracker::ReadCSV(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Failed to open CSV file: " << filename << std::endl;
            return;
        }

        impl_->path.poses.clear();
        impl_->path.header.frame_id = "map"; 

        std::string line;

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string item;

            double x, y;

            std::getline(ss, item, ',');
            x = std::stod(item);

            std::getline(ss, item, ',');
            y = std::stod(item);

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = x;
            pose.pose.position.y = y;

            impl_->path.poses.push_back(pose);
        }

        std::cout << "Loaded " << impl_->path.poses.size() << " waypoints from CSV into nav_msgs::Path." << std::endl;
    }

    geometry_msgs::PoseStamped GPStracker::findTarget(const geometry_msgs::Pose2D &pose, size_t &target_index, float &curr_dist) {
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

    morai_msgs::CtrlCmd GPStracker::Stanley(const geometry_msgs::Pose2D &pose, geometry_msgs::PoseStamped &target, float &curr_vel)
    {
        // --- Start by assuming the first waypoint is the closest ---
        size_t target_index = 0;
        float curr_dist = 0.0;
        target = GPStracker::findTarget(pose, target_index, curr_dist);

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
        float steering_angle = (heading_error + std::atan2(impl_->k * cross_track_error, impl_->vel));

        // Set the command values
        impl_->cmd.accel = std::max(std::min(PID(curr_vel, impl_->vel), 1.0f), 0.0f);
        impl_->cmd.steering = steering_angle;
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

    float GPStracker::PID(float current_speed, float target_speed)
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
        return output;
    }
}
