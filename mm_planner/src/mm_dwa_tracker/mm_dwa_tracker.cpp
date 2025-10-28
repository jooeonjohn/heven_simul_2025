#include "mm_dwa_tracker/mm_dwa_tracker.h"
#include "mm_gps_tracker/gps_tracker.h"

namespace dwa_tracker {

    struct dwaTracker::Impl {
        std::string pathfile;
        nav_msgs::OccupancyGrid ogm_map;
        nav_msgs::Path path;
        morai_msgs::CtrlCmd cmd;
        float lad, max_speed, min_speed, max_omega, min_omega;
        float dt, window, goal_weight, obs_weight;
        int filter_window_size;
        float goal_x, goal_y, resolution;
        float center_offset;
        gps_tracker::GPStracker gps_tracker;
    };

    dwaTracker::dwaTracker() : impl_(new Impl){
        tfListener_ = std::make_unique<tf2_ros::TransformListener>(tfBuffer_);
    }
    dwaTracker::~dwaTracker() {}

    void dwaTracker::Init(const std::string& pathfile, float lad, float max_speed, float min_speed, float max_omega, 
                   float min_omega, float dt, float window, float goal_weight, float obs_weight) 
    {
        impl_->lad = lad;
        impl_->dt = dt;
        impl_->window = window;
        impl_->goal_weight = goal_weight;
        impl_->obs_weight = obs_weight;
        impl_->resolution = 0.1;
        impl_->pathfile = pathfile;

        impl_->max_speed = max_speed;
        impl_->min_speed = min_speed;
        impl_->max_omega = max_omega;
        impl_->min_omega = min_omega;

        
        impl_->center_offset = 7.0/impl_->resolution;

        impl_->path = impl_->gps_tracker.Init(impl_->pathfile, impl_->lad, 3.0, 0, 0.3, 0.0, 0.01);

        impl_->cmd.brake = 1.0;
        impl_->cmd.accel = 0.0;
        impl_->cmd.steering = 0.0;
    }

    geometry_msgs::PoseStamped dwaTracker::transformationToBaseLink(const geometry_msgs::PoseStamped &target_map_frame) {
        geometry_msgs::PoseStamped target_base_link;
        try {
            tfBuffer_.transform(target_map_frame, target_base_link, "base_link", ros::Duration(0.5));
        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(2.0, "Failed to transform target pose to base_link frame: %s", ex.what());
            // return an "invalid" PoseStamped: set a flag by nan-ing the x to detect later
            target_base_link.pose.position.x = std::numeric_limits<double>::quiet_NaN();
            return target_base_link;
        }
        return target_base_link;
    }

    morai_msgs::CtrlCmd dwaTracker::dynamicWindowApproach(geometry_msgs::Pose2D &pose, 
        const nav_msgs::OccupancyGrid::ConstPtr &ogm, nav_msgs::Path &dwa_path, geometry_msgs::PoseStamped &target, float current_speed) {
        size_t target_index = 0;
        float curr_dist = 0;
        // Generate velocity samples within the dynamic window
        float min_v = impl_->min_speed;
        float max_v = impl_->max_speed;
        float min_theta = impl_->min_omega;
        float max_theta = impl_->max_omega;

        float best_score = -1e8;
        nav_msgs::Path best_path;

        geometry_msgs::PoseStamped target_map_frame;
        geometry_msgs::PoseStamped target_base_link;

        target_map_frame = impl_->gps_tracker.findTarget(pose, target_index, curr_dist);
        target_base_link = dwaTracker::transformationToBaseLink(target_map_frame);

        impl_->goal_x = target_base_link.pose.position.x / impl_->resolution;
        impl_->goal_y = -(target_base_link.pose.position.y / impl_->resolution) + impl_->center_offset; // Transform to OGM coordinate
        impl_->ogm_map = *ogm;

        // Evaluate each velocity and angular velocity pair
        for (float v = min_v; v <= max_v; v += 0.1) {
            for (float theta = min_theta; theta <= max_theta; theta += 0.5) {
                // Simulate trajectory for each (v, omega) pair
                dwaTracker::PathOutput path_info = simulateTrajectory(v, theta);

                // If the score is better, update the best trajectory
                if (path_info.score > best_score) {
                    best_score = path_info.score;
                    best_path = path_info.path; 

                    float raw_accel = std::max(std::min(impl_->gps_tracker.PID(current_speed, v), 1.0f), 0.0f);
                    float raw_steer = std::max(std::min(-theta, 40.0f), -40.0f)/(40);

                    // Update the command with the filtered values
                    impl_->cmd.accel = raw_accel;
                    impl_->cmd.steering = raw_steer;
                }
            }
        }
        impl_->cmd.brake = 0;
        dwa_path = best_path;
        target = target_map_frame;

        ROS_INFO("Final DRIVE CONTROL %f %f", 
            impl_->cmd.accel, 
            impl_->cmd.steering);

        return impl_->cmd;
    }

    dwaTracker::PathOutput dwaTracker::simulateTrajectory(float v, float theta) {

        // Simulate the trajectory for the given velocity and angular velocity
        dwaTracker::PathOutput control_info;
        float x = 0.0;
        float y = 0.0;
        float car_theta = theta*M_PI/180;
        float score = 0.0;

        nav_msgs::Path path;
        path.header.frame_id = "base_link"; // Set the coordinate frame
        path.header.stamp = ros::Time::now(); // Set the timestamp

        for (float t = 0.0; t <= impl_->window; t+= impl_->dt) {

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "base_link";
            pose.header.stamp = ros::Time::now();
            float v_ms = v * 5/18;
            // Update the car's pose using forward kinematic
            x += v_ms * cos(car_theta) * impl_->dt;
            y += v_ms * sin(car_theta) * impl_->dt;

            score -= isCollision(x, y);
            score -= evaluateTrajectory(x, y, car_theta, v);

            pose.pose.position.x = x * impl_->resolution - 1.5;
            pose.pose.position.y = y * -impl_->resolution;
            pose.pose.position.z = 0.0;

            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }

        control_info.score = score;
        control_info.path = path;
        return control_info;
    }

    float dwaTracker::isCollision(float x, float y) {
        // Convert the world coordinates (x, y) to grid coordinates
        int grid_x = static_cast<int>(x);
        int grid_y = static_cast<int>(-y + impl_->center_offset);

        // Checking the boundary of the local map
        if (grid_x < 0 || grid_x >= impl_->ogm_map.info.width || grid_y < 0 || grid_y >= impl_->ogm_map.info.height) {
            return 10000.0;
        }

        // Check if the grid cell is occupied
        int index = grid_y * impl_->ogm_map.info.width + grid_x;

        if (impl_->ogm_map.data[index] > 0) {  // Threshold for occupied cells
            // ROS_INFO("------- Obstacle -------");
            return impl_->ogm_map.data[index] * impl_->obs_weight;
        }
        return 0;
    }

    float dwaTracker::evaluateTrajectory(float x, float y, float theta, float v) {
        int grid_x = static_cast<int>(std::lround(x));
        int grid_y = static_cast<int>(std::lround(y + impl_->center_offset));
        float dx = static_cast<float>(impl_->goal_x) - static_cast<float>(grid_x);
        float dy = static_cast<float>(impl_->goal_y) - static_cast<float>(grid_y);
        float distance = sqrt(dx*dx + dy*dy);
        float distance_to_goal = impl_->goal_weight * distance;
        // if (grid_y > impl_->center_offset) distance_to_goal -= 2.0f;
        return distance_to_goal;
    }

}
