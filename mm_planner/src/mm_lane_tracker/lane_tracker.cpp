#include "mm_lane_tracker/lane_tracker.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

namespace lane_tracker
{
    struct Lanetracker::Impl
    {
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

    Lanetracker::Lanetracker() : impl_(new Impl){}
    Lanetracker::~Lanetracker() {}

    void Lanetracker::Init(float vel, float k, float kp, float ki, float kd)
    {
        impl_->vel = vel;
        impl_->k = k;

        impl_->cmd.brake = 1.0;
        impl_->cmd.accel = 0.0;
        impl_->cmd.steering = 0.0;

        impl_->kp = kp;
        impl_->ki = ki;
        impl_->kd = kd;
    }

    morai_msgs::CtrlCmd Lanetracker::Stanley(mm_common::lane_info lane, float &curr_vel)
    {
        // Calculate the heading error
        float avg_theta = (lane.left_theta+lane.left_theta)/2;
        float heading_error = (avg_theta-90.0) * M_PI / 180.0;
        // Normalize the heading error to [-pi, pi]
        while (heading_error > M_PI) heading_error -= 2 * M_PI;
        while (heading_error < -M_PI) heading_error += 2 * M_PI;
        // Calculate the cross-track error
        float left_x_f = static_cast<float>(lane.left_x);
        float right_x_f = static_cast<float>(lane.right_x);
        float cross_track_error = (left_x_f / (left_x_f + right_x_f) - 0.5f) * 2.0f;
        // Calculate the steering angle using the Stanley controller formula
        float steering_angle = (heading_error + std::atan2(impl_->k * cross_track_error, impl_->vel/3.6)) * 0.7; // use m/s

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

    float Lanetracker::PID(float current_speed, float target_speed)
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
