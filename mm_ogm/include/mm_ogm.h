#ifndef LOCAL_MAP_BUILDER_H
#define LOCAL_MAP_BUILDER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <limits>

namespace local_ogm_builder {

    class LocalOGMBuilder {
    public:
        explicit LocalOGMBuilder(ros::NodeHandle& nh);
        void poseCallback(const geometry_msgs::Pose2D::ConstPtr &msg);
        void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void laneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void timerCallback(const ros::TimerEvent&);

    private:
        void transformCloudToLocalFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);
        void updateLocalMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& new_cloud);
        void updateOccupancyGrid();
        void moveOccupancyGridWithLocal();
        void updateInflationRadius(int start_x, int start_y);
        void publishOccupancyGrid();

        // ROS
        ros::NodeHandle nh_;
        ros::Subscriber pose_sub_;
        ros::Subscriber lidar_sub_;
        ros::Subscriber lane_sub_;
        ros::Publisher local_map_pub_;
        ros::Publisher global_map_pub_;
        ros::Publisher occupancy_grid_pub_;
        ros::Publisher path_pub_;
        ros::Timer timer_;

        // TF
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // Internal data
        geometry_msgs::Pose2D current_pose_;
        geometry_msgs::Pose2D previous_pose_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_;
        nav_msgs::OccupancyGrid occupancy_grid_;
        std::vector<std::chrono::steady_clock::time_point> occupied_timestamps_;
        nav_msgs::Path path_;
        sensor_msgs::PointCloud2 global_map_msg_;
        ros::Duration persistence_duration_;

        // Parameters
        double resolution_;
        double update_frequency_;
    
        struct GridCell {
            int x, y;
            int distance;
        };

        struct LanePoint {
            float x;
            float y;
        };

        std::vector<LanePoint> lane_points_;
        bool has_lane_points_;
        bool lane_flag_active_;

        double x_max, x_min, y_max, y_min;
        int inflation_range;
        float inflation_weight;
    };
}

#endif // LOCAL_MAP_BUILDER_H
