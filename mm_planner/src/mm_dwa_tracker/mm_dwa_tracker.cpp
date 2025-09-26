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
        gps_tracker::GPStracker gps_tracker;
    };

    dwaTracker::dwaTracker() : impl_(new Impl){
        tfListener_ = std::make_unique<tf2_ros::TransformListener>(tfBuffer_);
    }
    dwaTracker::~dwaTracker() {}

    void dwaTracker::Init(const std::string& pathfile, float lad, float max_speed, float min_speed, float max_omega, 
                   float min_omega, float dt, float window, float goal_weight, float obs_weight, int filter_window_size) 
    {
        impl_->lad = 4.5;
        impl_->dt = 1.5;
        impl_->window = 40.0;
        impl_->goal_weight = 0.7;
        impl_->obs_weight = 300.0;
        impl_->filter_window_size = 5;
        float min_v = 3.0;
        float max_v = 4.0;
        float min_theta = -45.0;
        float max_theta = 55.0;

        impl_->pathfile = pathfile;
        impl_->lad = lad;
        impl_->max_speed = max_speed;
        impl_->min_speed = min_speed;
        impl_->max_omega = max_omega;
        impl_->min_omega = min_omega;
        impl_->dt = dt;
        impl_->window = window;
        impl_->goal_weight = goal_weight;
        impl_->obs_weight = obs_weight;
        impl_->filter_window_size = filter_window_size;
        impl_->resolution = 0.1;

        impl_->path = impl_->gps_tracker.Init(impl_->pathfile, impl_->lad, 3.0, 0, 0.0, 0.0, 0.0);

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
        const nav_msgs::OccupancyGrid::ConstPtr &ogm, nav_msgs::Path &dwa_path, geometry_msgs::PoseStamped &target) {
        size_t target_index = 0;
        float curr_dist = 0;
        // Generate velocity samples within the dynamic window
        float min_v = 3.0;
        float max_v = 4.0;
        float min_theta = -45.0;
        float max_theta = 55.0;

        float best_score = -1e8;
        nav_msgs::Path best_path;

        geometry_msgs::PoseStamped target_map_frame;
        geometry_msgs::PoseStamped target_base_link;

        target_map_frame = impl_->gps_tracker.findTarget(pose, target_index, curr_dist);
        target_base_link = dwaTracker::transformationToBaseLink(target_map_frame);

        impl_->goal_x = target_base_link.pose.position.x / impl_->resolution;
        impl_->goal_y = -(target_base_link.pose.position.y / impl_->resolution) + 50; // Transform to OGM coordinate
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
                    float raw_speed = std::max(std::min(v * 10.0, 1000.0), 0.0);
                    float raw_steer = std::max(std::min(theta, 28.0f), -28.0f) * 71;

                    // Apply the moving average filter
                    applyMovingAverageFilter(raw_speed, raw_steer);

                    // Update the command with the filtered values
                    impl_->cmd.accel = static_cast<int>(raw_speed);
                    impl_->cmd.steering = static_cast<int>(raw_steer);
                }
            }
        }
        impl_->cmd.brake = 0;
        dwa_path = best_path;
        target = target_base_link;
        int pose_idx = best_path.poses.size()-1;

        ROS_INFO_THROTTLE(1.0, "the position of global goal %f %f", 
            target_map_frame.pose.position.x, target_map_frame.pose.position.y);

        ROS_INFO_THROTTLE(1.0, "the position of local goal %f %f", 
            impl_->goal_x, impl_->goal_y);

        ROS_INFO_THROTTLE(1.0, "-----Position (x, y): %f, %f", 
            best_path.poses[pose_idx].pose.position.x * 10.0, 
            -best_path.poses[pose_idx].pose.position.y * 10.0 + 50.0);

        ROS_INFO_THROTTLE(1.0, "-----Best Score: %f", 
            best_score);

        ROS_INFO_THROTTLE(1.0, "Final DRIVE CONTROL %d %d", 
            impl_->cmd.accel, 
            impl_->cmd.steering);

        ROS_INFO_THROTTLE(1.0, "Final frame ID: %s", 
            best_path.header.frame_id.c_str());

        
        return impl_->cmd;
    }

    void dwaTracker::applyMovingAverageFilter(float& speed, float& steer) {
        // assume speed_history & steer_history are std::deque<float> members
        speed_history.push_back(speed);
        steer_history.push_back(steer);

        while (static_cast<int>(speed_history.size()) > impl_->filter_window_size) speed_history.pop_front();
        while (static_cast<int>(steer_history.size()) > impl_->filter_window_size) steer_history.pop_front();

        float sum_speed = 0.0f;
        for (float v : speed_history) sum_speed += v;
        speed = sum_speed / static_cast<float>(speed_history.size());

        float sum_steer = 0.0f;
        for (float s : steer_history) sum_steer += s;
        steer = sum_steer / static_cast<float>(steer_history.size());
    }

    dwaTracker::PathOutput dwaTracker::simulateTrajectory(float v, float theta) {

        // Simulate the trajectory for the given velocity and angular velocity
        dwaTracker::PathOutput control_info;
        float x = 15.0;
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

            pose.pose.position.x = x * impl_->resolution;
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
        int grid_y = static_cast<int>(-y + 50.0);

        // Checking the boundary of the local map
        if (grid_x < 0 || grid_x >= impl_->ogm_map.info.width || grid_y < 0 || grid_y >= impl_->ogm_map.info.height) {
            return 10000.0;
        }

        // Check if the grid cell is occupied
        int index = grid_y * impl_->ogm_map.info.width + grid_x;
        // ROS_INFO("Checking OBS index: %d, x: %d, y: %d, OBS VAL: %d", index, grid_x, grid_y, impl_->ogm_map.data[index]);
        if (impl_->ogm_map.data[index] > 90) {  // Threshold for occupied cells
            // ROS_INFO("------- Obstacle -------");
            return impl_->ogm_map.data[index] * impl_->obs_weight;
        }
        return 0;
    }

    float dwaTracker::evaluateTrajectory(float x, float y, float theta, float v) {
        int grid_x = static_cast<int>(std::lround(x));
        int grid_y = static_cast<int>(std::lround(y + 50.0f));
        float dx = static_cast<float>(impl_->goal_x) - static_cast<float>(grid_x);
        float dy = static_cast<float>(impl_->goal_y) - static_cast<float>(grid_y);
        float distance = sqrt(dx*dx + dy*dy);
        float distance_to_goal = impl_->goal_weight * distance;
        if (grid_y > 50) distance_to_goal -= 2.0f;
        return distance_to_goal;
    }

}
