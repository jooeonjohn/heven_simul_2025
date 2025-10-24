#include "mm_planner/planner.h"

namespace planner
{
    struct Planner::Impl
    {
        std::string pathfile, mission_points_path;
        float dist_threshold;

        //gps tracker param
        float lad, vel, stanley_k, pid_kp, pid_ki, pid_kd;

        //dwa tracker param
        float dwa_lad, dwa_max_speed, dwa_min_speed, dwa_max_omega, dwa_min_omega;
        float dwa_dt, dwa_window, dwa_goal_weight, dwa_obs_weight;
        int dwa_filter_window_size;

        ros::Subscriber sub_pose;
        ros::Subscriber sub_vel;
        ros::Subscriber sub_ogm;

        ros::Publisher pub_command;
        ros::Publisher pub_path;
        ros::Publisher pub_target;
        ros::Publisher pub_dwa_path;
        std::vector<std::array<double, 4>> missions;
        int current_mission;
        bool mission_on;

        nav_msgs::Path path;
        ros::Timer timer;

        nav_msgs::OccupancyGrid::ConstPtr ogm;
        geometry_msgs::Pose2D pose;
        float curr_vel;
        morai_msgs::CtrlCmd cmd;

        gps_tracker::GPStracker gps_tracker;
        dwa_tracker::dwaTracker dwa_tracker;

    };

    Planner::Planner() : impl_(new Impl) {}
    Planner::~Planner() {}

    void Planner::Init(ros::NodeHandle &nh)
    {
        ROS_ASSERT(nh.getParam("path", impl_->pathfile));
        ROS_ASSERT(nh.getParam("mission_points_path", impl_->mission_points_path));
        ROS_ASSERT(nh.getParam("dist_threshold", impl_->dist_threshold));
        ROS_ASSERT(nh.getParam("current_mission", impl_->current_mission));
        ROS_ASSERT(nh.getParam("mission_on", impl_->mission_on));

        ROS_ASSERT(nh.getParam("lad", impl_->lad));
        ROS_ASSERT(nh.getParam("vel", impl_->vel));
        ROS_ASSERT(nh.getParam("stanley_k", impl_->stanley_k));
        ROS_ASSERT(nh.getParam("pid_kp", impl_->pid_kp));
        ROS_ASSERT(nh.getParam("pid_ki", impl_->pid_ki));
        ROS_ASSERT(nh.getParam("pid_kd", impl_->pid_kd));

        ROS_ASSERT(nh.getParam("dwa_lad", impl_->dwa_lad));
        ROS_ASSERT(nh.getParam("dwa_max_speed", impl_->dwa_max_speed));
        ROS_ASSERT(nh.getParam("dwa_min_speed", impl_->dwa_min_speed));
        ROS_ASSERT(nh.getParam("dwa_max_omega", impl_->dwa_max_omega));
        ROS_ASSERT(nh.getParam("dwa_min_omega", impl_->dwa_min_omega));
        ROS_ASSERT(nh.getParam("dwa_dt", impl_->dwa_dt));
        ROS_ASSERT(nh.getParam("dwa_window", impl_->dwa_window));
        ROS_ASSERT(nh.getParam("dwa_goal_weight", impl_->dwa_goal_weight));
        ROS_ASSERT(nh.getParam("dwa_obs_weight", impl_->dwa_obs_weight));
        ROS_ASSERT(nh.getParam("dwa_filter_window_size", impl_->dwa_filter_window_size));

        ReadMissionPoints(impl_->mission_points_path);

        // initialize subscriber
        impl_->sub_pose = nh.subscribe("pose", 2,  &Planner::PoseCallback, this);
        impl_->sub_vel = nh.subscribe("/Competition_topic", 1,  &Planner::VelocityCallback, this);
        impl_->sub_ogm = nh.subscribe<nav_msgs::OccupancyGrid>("ogm", 2, &Planner::ogmCallback, this);
        
        // initialize publishers
        impl_->pub_command = nh.advertise<morai_msgs::CtrlCmd>("command", 1);
        impl_->pub_path = nh.advertise<nav_msgs::Path>("/mm_planner/path", 1);
        impl_->pub_target = nh.advertise<visualization_msgs::Marker>("/mm_planner/target", 1);
        impl_->pub_dwa_path = nh.advertise<nav_msgs::Path>("/mm_planner/dwa_path", 1);
         
        impl_->path = impl_->gps_tracker.Init(impl_->pathfile, impl_->lad, impl_->vel, impl_->stanley_k, impl_->pid_kp, impl_->pid_ki, impl_->pid_kd);
        impl_->dwa_tracker.Init(impl_->pathfile, impl_->dwa_lad, impl_->dwa_max_speed, impl_->dwa_min_speed,
            impl_->dwa_max_omega, impl_->dwa_min_omega, impl_->dwa_dt, impl_->dwa_window,
            impl_->dwa_goal_weight, impl_->dwa_obs_weight, impl_->dwa_filter_window_size
        );

        impl_->timer = nh.createTimer(ros::Duration(0.1), &Planner::PublishPathCallback, this);

    }

    void Planner::ReadMissionPoints(const std::string &filename)
    {
        // Open the mission points CSV file
        std::ifstream file(filename);
        if (!file.is_open())
        {
            ROS_ERROR("Cannot open mission points file: %s", filename.c_str());
            return;
        }

        std::string line;
        // Read the file line by line
        while (std::getline(file, line))
        {
            // Skip empty lines
            if (line.empty()) {
                continue;
            }

            std::stringstream ss(line);
            std::array<double, 4> current_mission;
            std::string value;

            // Parse the comma-separated values
            // Format: start_x, start_y, end_x, end_y
            try {
                // Index 0: Mission Start X
                std::getline(ss, value, ',');
                current_mission[0] = std::stod(value);

                // Index 1: Mission Start Y
                std::getline(ss, value, ',');
                current_mission[1] = std::stod(value);

                // Index 2: Mission End X
                std::getline(ss, value, ',');
                current_mission[2] = std::stod(value);

                // Index 3: Mission End Y
                std::getline(ss, value); // Last value on the line
                current_mission[3] = std::stod(value);

                // Add the parsed mission to our vector
                impl_->missions.push_back(current_mission);
            }
            catch (const std::invalid_argument& ia) {
                ROS_WARN("Invalid argument while parsing mission points line: %s", line.c_str());
            }
            catch (const std::out_of_range& oor) {
                ROS_WARN("Out of range while parsing mission points line: %s", line.c_str());
            }
        }
        
        file.close();
        ROS_INFO("%zu missions loaded successfully from %s", impl_->missions.size(), filename.c_str());
    }

    void Planner::MainPlanning()
    {
        visualization_msgs::Marker target_mk;
        geometry_msgs::PoseStamped target;
        nav_msgs::Path dwa_path;

        float start_x, start_y, end_x, end_y;
        start_x = impl_->missions[impl_->current_mission][0];
        start_y = impl_->missions[impl_->current_mission][1];
        end_x = impl_->missions[impl_->current_mission][2];
        end_y = impl_->missions[impl_->current_mission][3];
        
        float start_dist = std::hypot(start_x-impl_->pose.x, start_y-impl_->pose.y);
        float end_dist = std::hypot(end_x-impl_->pose.x, end_y-impl_->pose.y);
        std::cout << "points : " << start_x << ", " << start_y << ", " << end_x << ", " << end_y << std::endl;
        std::cout << "start_dist : " << start_dist << ", end_dist : " << end_dist << std::endl;

        if (start_dist < impl_->dist_threshold) impl_->mission_on = true;
        if (impl_->mission_on && end_dist < impl_->dist_threshold) {
            impl_->current_mission++;
            impl_->mission_on = false;
        }
         
        if (impl_->mission_on){
            
            impl_->cmd = impl_->gps_tracker.Stanley(impl_->pose, target, impl_->curr_vel);
            
        }
        else impl_->cmd = impl_->gps_tracker.Stanley(impl_->pose, target, impl_->curr_vel);

        target_mk = PublishTargetMarker(target, "map");
        
    }

    void Planner::ogmCallback(const nav_msgs::OccupancyGrid::ConstPtr &ogm_msg) 
    {
        impl_->ogm = ogm_msg;
    }

    void Planner::PoseCallback(const geometry_msgs::Pose2D &msg)
    {
        impl_->pose = msg;
        MainPlanning();
        std::cout << "Current mission : " << impl_->current_mission << impl_->mission_on << std::endl;
        // Publish the command
        impl_->pub_command.publish(impl_->cmd);
    }

    void Planner::VelocityCallback(const morai_msgs::EgoVehicleStatus &msg)
    {   
        float x_vel = msg.velocity.x;
        impl_->curr_vel = x_vel*3.6; // km/h
    }

    void Planner::PublishPathCallback(const ros::TimerEvent&)
    {
        impl_->pub_path.publish(impl_->path);
    }

    visualization_msgs::Marker Planner::PublishTargetMarker(geometry_msgs::PoseStamped target, std::string frame_id) 
    {
        visualization_msgs::Marker target_mk;
        target_mk.header.frame_id = frame_id;
        target_mk.header.stamp = ros::Time::now();
        target_mk.type = visualization_msgs::Marker::SPHERE;

        // Position from target waypoint
        target_mk.pose.position.x = target.pose.position.x;
        target_mk.pose.position.y = target.pose.position.y;
        target_mk.pose.position.z = 0.0;

        // Scale for the sphere
        target_mk.scale.x = 1.0;
        target_mk.scale.y = 1.0;
        target_mk.scale.z = 1.0;

        // Color red
        target_mk.color.a = 1.0;
        target_mk.color.r = 1.0;
        target_mk.color.g = 0.0;
        target_mk.color.b = 0.0;
        
        impl_->pub_target.publish(target_mk);
        
        return target_mk;
    }

}
