#include "mm_ogm.h"
#include <cmath>


namespace local_ogm_builder {

    LocalOGMBuilder::LocalOGMBuilder(ros::NodeHandle& nh) 
        : nh_(nh), tf_listener_(tf_buffer_), update_frequency_(10.0) { // Default update frequency set to 10 Hz
        local_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        has_lane_points_ = false;
        lane_flag_active_ = false;
        
        ROS_ASSERT(nh.getParam("x_max", x_max));
        ROS_ASSERT(nh.getParam("x_min", x_min));
        ROS_ASSERT(nh.getParam("y_max", y_max));
        ROS_ASSERT(nh.getParam("y_min", y_min));
        ROS_ASSERT(nh.getParam("inflation_range", inflation_range));
        ROS_ASSERT(nh.getParam("inflation_weight", inflation_weight));

        pose_sub_ = nh_.subscribe("pose", 1, &LocalOGMBuilder::poseCallback, this);
        lidar_sub_ = nh_.subscribe("lidar", 1, &LocalOGMBuilder::lidarCallback, this);
        lane_sub_ = nh_.subscribe("/lane_output", 1, &LocalOGMBuilder::laneCallback, this);

        occupancy_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("ogm", 1);

        resolution_ = 0.1; // Grid resolution in meters per cell
        previous_pose_ = current_pose_; // Initialize previous_pose_ with the current_pose_

        timer_ = nh_.createTimer(ros::Duration(1.0 / update_frequency_), &LocalOGMBuilder::timerCallback, this); // Timer for periodic updates
    }

    void LocalOGMBuilder::poseCallback(const geometry_msgs::Pose2D::ConstPtr &msg) {
        current_pose_ = *msg;

        // Add current pose to path
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(0);
        pose_stamped.header.frame_id = "base_link";
        pose_stamped.pose.position.x = current_pose_.x;
        pose_stamped.pose.position.y = current_pose_.y;

        path_.header.stamp = ros::Time(0);
        path_.header.frame_id = "base_link";
        path_.poses.push_back(pose_stamped);

        // Optionally, limit the number of path points for performance
        if (path_.poses.size() > 1000) {
            path_.poses.erase(path_.poses.begin()); // Keep the latest 1000 points
        }
    }

    void LocalOGMBuilder::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        transformCloudToLocalFrame(cloud, transformed_cloud);

        if (transformed_cloud && !transformed_cloud->empty()) {
            updateLocalMap(transformed_cloud);
        } else {
            ROS_WARN("Transformed cloud is empty or invalid.");
        }
    }

    void LocalOGMBuilder::transformCloudToLocalFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {
        geometry_msgs::TransformStamped transform_stamped;
        try {
            // Lookup the transform from the point cloud frame to the base_link frame
            transform_stamped = tf_buffer_.lookupTransform("base_link", input_cloud->header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Transform the point cloud to the base_link frame using the found transformation
        pcl_ros::transformPointCloud(*input_cloud, *output_cloud, transform_stamped.transform);
        output_cloud->header.frame_id = "base_link";
    }

    void LocalOGMBuilder::updateLocalMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& new_cloud) {
        // Add new points to local map
        *local_map_ = *new_cloud;
        // ROS_INFO("Local map now has %lu points", local_map_->points.size());

        // Remove points outside the max_distance_ boundary
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(local_map_);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(x_min, x_max);
        pass.filter(*local_map_);

        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_min, y_max);
        pass.filter(*local_map_);

        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.1, 0.7);
        pass.filter(*local_map_);
        updateOccupancyGrid();
    }

    void LocalOGMBuilder::updateOccupancyGrid() {
        // Occupancy Grid Size
        occupancy_grid_.info.resolution = resolution_;
        occupancy_grid_.info.width = static_cast<int>((x_max - x_min) / resolution_);
        occupancy_grid_.info.height = static_cast<int>((y_max - y_min) / resolution_);
        occupancy_grid_.info.origin.position.x = x_min;
        occupancy_grid_.info.origin.position.y = y_min;
        occupancy_grid_.info.origin.position.z = 0.0;
        occupancy_grid_.info.origin.orientation.w = 1.0;
        occupancy_grid_.data.assign(occupancy_grid_.info.width * occupancy_grid_.info.height, -1);

        // Occupancy Grid Update based on Local_map
        for (const auto& point : local_map_->points) {
            int grid_x = static_cast<int>((point.x - x_min) / resolution_);
            int grid_y = static_cast<int>((point.y - y_min) / resolution_);

            if (grid_x >= 0 && grid_x < occupancy_grid_.info.width && grid_y >= 0 && grid_y < occupancy_grid_.info.height) {
                int index = grid_y * occupancy_grid_.info.width + grid_x;
                if(grid_y >= 60 && grid_y <= 80 && grid_x >= 0 && grid_x <= 15) continue;
                occupancy_grid_.data[index] = 100; // Occupied cell
                updateInflationRadius(grid_x, grid_y);
            }
        }

    }

    void LocalOGMBuilder::moveOccupancyGridWithLocal() {
        // Compute the gps point change
        double dx = current_pose_.x - previous_pose_.x;
        double dy = current_pose_.y - previous_pose_.y;

        // Transform the points
        std::vector<int8_t> new_data(occupancy_grid_.data.size(), -1);
        for (int y = 0; y < occupancy_grid_.info.height; ++y) {
            for (int x = 0; x < occupancy_grid_.info.width; ++x) {
                int index = y * occupancy_grid_.info.width + x;

                // Calculate based on base_link
                int new_x = x - static_cast<int>(dx / resolution_);
                int new_y = y - static_cast<int>(dy / resolution_);

                if (new_x >= 0 && new_x < occupancy_grid_.info.width && new_y >= 0 && new_y < occupancy_grid_.info.height) {
                    int new_index = new_y * occupancy_grid_.info.width + new_x;
                    new_data[new_index] = occupancy_grid_.data[index];
                }
            }
        }

        occupancy_grid_.data = new_data;

        // Update the previous pose
        previous_pose_ = current_pose_;
    }

    void LocalOGMBuilder::updateInflationRadius(int start_x, int start_y) {
        int dx[4] = {0,0,1,-1};
        int dy[4] = {1,-1,0,0};
        int front = 0;
        int rear = -1;

        GridCell queue[20005] = {};
        queue[++rear] = {start_x, start_y, 0};

        int visited[50005] = {};
        visited[start_y * occupancy_grid_.info.width + start_x] = 1;

        while (front <= rear) {
            GridCell current = queue[front++];

            if(current.distance >= inflation_range) continue;
            if(front > 20000) continue;
            if(current.y >= 65 && current.y <= 75 && current.x >= 5 && current.x <= 17) continue;

            for (int dir = 0; dir < 4; dir++) {
                if(rear > 20000) return;

                int nx = current.x + dx[dir];
                int ny = current.y + dy[dir];
                int index = ny * occupancy_grid_.info.width + nx;

                if(visited[index]) continue;
                if (nx < 0 || nx >= occupancy_grid_.info.width || ny < 0 || ny >= occupancy_grid_.info.height) continue;

                int cost = 100 - inflation_weight * current.distance;
                if(occupancy_grid_.data[index] < cost) occupancy_grid_.data[index] = static_cast<int8_t>(cost);//Decreasing from 100, 90 80 70, inflation radius
                visited[index] = 1;
                queue[++rear] = {nx, ny, current.distance + 1};
                
            }
        }
    }

    void LocalOGMBuilder::timerCallback(const ros::TimerEvent&) {
        // moveOccupancyGridWithLocal();
        publishOccupancyGrid();
    }

    void LocalOGMBuilder::laneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        lane_points_.clear();
        lane_points_.reserve(cloud.points.size());

        for (const auto& point : cloud.points) {
            lane_points_.push_back({point.x, point.y});
        }

        has_lane_points_ = !lane_points_.empty();
    }

    void LocalOGMBuilder::publishOccupancyGrid() {
        occupancy_grid_.header.frame_id = "base_link";
        occupancy_grid_.header.stamp = ros::Time(0);
        occupancy_grid_pub_.publish(occupancy_grid_);
    }

}
