#include "mm_planner/planner.h"

namespace planner
{
    struct Planner::Impl
    {
        std::string pathfile, mission_points_path;
        float dist_threshold, traffic_dist_threshold, slow_dist_threshold;

        //gps tracker param
        float lad, vel, stanley_k, pid_kp, pid_ki, pid_kd;

        //dwa tracker param
        float dwa_lad, dwa_max_speed, dwa_min_speed, dwa_max_omega, dwa_min_omega;
        float dwa_dt, dwa_window, dwa_goal_weight, dwa_obs_weight;
        int dwa_filter_window_size;
        int traffic;

        ros::Subscriber sub_pose;
        ros::Subscriber sub_vel;
        ros::Subscriber sub_ogm;
        ros::Subscriber sub_collision;
        ros::Subscriber sub_lane;
        ros::Subscriber sub_traffic;

        ros::Publisher pub_command;
        ros::Publisher pub_path;
        ros::Publisher pub_target;
        ros::Publisher pub_dwa_path;
        std::vector<std::array<double, 4>> missions;
        std::vector<std::array<double, 2>> traffic_points;
        std::vector<std::array<double, 2>> slow_points;
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
        cut_in::CutIn cut_in;
        lane_tracker::Lanetracker lane_tracker;

        bool collision;
        bool reversing = false;
        ros::Time reverse_start_time;

        mm_common::lane_info lane;

    };

    Planner::Planner() : impl_(new Impl) {}
    Planner::~Planner() {}

    void Planner::Init(ros::NodeHandle &nh)
    {
        ROS_ASSERT(nh.getParam("path", impl_->pathfile));
        ROS_ASSERT(nh.getParam("mission_points_path", impl_->mission_points_path));
        ROS_ASSERT(nh.getParam("dist_threshold", impl_->dist_threshold));
        ROS_ASSERT(nh.getParam("traffic_dist_threshold", impl_->traffic_dist_threshold));
        ROS_ASSERT(nh.getParam("slow_dist_threshold", impl_->slow_dist_threshold));
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
        ReadMissionPoints(impl_->mission_points_path);

        // initialize subscriber
        impl_->sub_pose = nh.subscribe("pose", 2,  &Planner::PoseCallback, this);
        impl_->sub_vel = nh.subscribe("/Competition_topic", 1,  &Planner::VelocityCallback, this);
        impl_->sub_collision = nh.subscribe("/CollisionData", 1,  &Planner::CollisionCallback, this);
        impl_->sub_ogm = nh.subscribe<nav_msgs::OccupancyGrid>("ogm", 2, &Planner::ogmCallback, this);
        impl_->sub_lane = nh.subscribe("/lane_result", 1, &Planner::laneCallback, this);
        impl_->sub_traffic = nh.subscribe("/traffic_signal", 1, &Planner::trafficCallback, this);
        
        // initialize publishers
        impl_->pub_command = nh.advertise<morai_msgs::CtrlCmd>("command", 1);
        impl_->pub_path = nh.advertise<nav_msgs::Path>("/mm_planner/path", 1);
        impl_->pub_target = nh.advertise<visualization_msgs::Marker>("/mm_planner/target", 1);
        impl_->pub_dwa_path = nh.advertise<nav_msgs::Path>("/mm_planner/dwa_path", 1);
         
        impl_->path = impl_->gps_tracker.Init(impl_->pathfile, impl_->lad, impl_->vel, impl_->stanley_k, impl_->pid_kp, impl_->pid_ki, impl_->pid_kd);
        impl_->dwa_tracker.Init(impl_->pathfile, impl_->dwa_lad, impl_->dwa_max_speed, impl_->dwa_min_speed,
            impl_->dwa_max_omega, impl_->dwa_min_omega, impl_->dwa_dt, impl_->dwa_window,
            impl_->dwa_goal_weight, impl_->dwa_obs_weight
        );
        impl_->cut_in.Init(impl_->path, impl_->lad, impl_->vel, impl_->stanley_k, impl_->pid_kp, impl_->pid_ki, impl_->pid_kd, 50, 800);
        impl_->lane_tracker.Init(impl_->vel, impl_->stanley_k, impl_->pid_kp, impl_->pid_ki, impl_->pid_kd);

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
            std::string type;
            std::string value;

            // First column = type (MISSION, TRAFFIC, RTURN, etc.)
            std::getline(ss, type, ',');

            try {
                if (type == "MISSION") {
                    std::array<double, 4> mission{};
                    std::getline(ss, value, ','); mission[0] = std::stod(value);
                    std::getline(ss, value, ','); mission[1] = std::stod(value);
                    std::getline(ss, value, ','); mission[2] = std::stod(value);
                    std::getline(ss, value);      mission[3] = std::stod(value);

                    impl_->missions.push_back(mission);
                }
                else if (type == "TRAFFIC") {
                    std::array<double, 2> traffic{};
                    std::getline(ss, value, ','); traffic[0] = std::stod(value);
                    std::getline(ss, value);      traffic[1] = std::stod(value);

                    impl_->traffic_points.push_back(traffic);
                }
                else if (type == "SLOWDWN") {
                    std::array<double, 2> slow{};
                    std::getline(ss, value, ','); slow[0] = std::stod(value);
                    std::getline(ss, value);      slow[1] = std::stod(value);

                    impl_->slow_points.push_back(slow);
                }
                else {
                    ROS_WARN("Unknown line type in mission file: %s", type.c_str());
                }
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
        bool is_slow = false;
        
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

        static bool has_stopped = false;  // 멈췄는지 확인하는 플래그
        for (const auto& tp : impl_->traffic_points) {
            float dist = std::hypot(tp[0] - impl_->pose.x, tp[1] - impl_->pose.y);
            

            // 특정 거리 안에 들어오면 멈춤
            if (dist < impl_->traffic_dist_threshold) {

                if (!has_stopped) {
                    if (impl_->curr_vel < 1.0){
                        has_stopped = true;  // 이미 멈춰있다면 플래그 설정
                        std::cout << "Traffic true" << std::endl;
                        return;
                    }

                    // 처음 멈출 때만 실행
                    std::cout << "Traffic stop" << std::endl;
                    impl_->cmd.brake = 1.0;
                    impl_->cmd.accel = 0.0;
                    impl_->cmd.steering = 0.0;
                    return;
                }
                
                // 현재 속도가 충분히 낮아졌을 때 traffic 값 확인
                if (impl_->traffic == 0 || impl_->traffic == -1) {
                    // traffic 값이 0이면 출발
                    std::cout << "Traffic signal is green. Proceeding." << std::endl;
                    impl_->cmd = impl_->gps_tracker.Stanley(impl_->pose, target, impl_->curr_vel, is_slow);
                    return;
                } 
                else {
                    // traffic 값이 유효하지 않은 경우 기본적으로 멈춤 유지
                    std::cout << "Traffic signal is red. Staying stopped." << std::endl;
                    impl_->cmd.brake = 1.0;
                    impl_->cmd.accel = 0.0;
                    impl_->cmd.steering = 0.0;
                    return;
                }
            }
            
        }
        has_stopped = false;

        
        for (const auto& sp : impl_->slow_points){
            float dist = std::hypot(sp[0]-impl_->pose.x, sp[1]-impl_->pose.y);
            
            if (dist < impl_->slow_dist_threshold) {
                std::cout << "Slow Down" << std::endl;
                is_slow = true;
            }
        }

        

        // ======================
        // Collision Handling
        // ======================
        if (impl_->collision && !impl_->reversing) {
            impl_->reversing = true;
            impl_->reverse_start_time = ros::Time::now();
            ROS_WARN("Collision detected! Reversing...");
        }

        if (impl_->reversing) {
            // Check how long we've been reversing
            double elapsed = (ros::Time::now() - impl_->reverse_start_time).toSec();
            if (elapsed < 2.0) {
                // Reverse: set velocity backward
                impl_->cmd.brake = 0.0;
                impl_->cmd.accel = -0.5;
                impl_->cmd.steering = 1.0;
                return;
            } 
            else if (elapsed < 3.0) {
                // Reverse: set velocity backward
                impl_->cmd.brake = 1.0;
                impl_->cmd.accel = 0.0;
                impl_->cmd.steering = 0.0;
                return;
            }
            else {
                // Done reversing
                impl_->reversing = false;
                impl_->collision = false; // reset collision flag
                ROS_INFO("Reverse complete, resuming planning");
            }
        }

        if (impl_->mission_on){
            if(impl_->current_mission == 0 || impl_->current_mission == 2) {
                impl_->cmd = impl_->dwa_tracker.dynamicWindowApproach(impl_->pose, impl_->ogm, dwa_path, target, impl_->curr_vel);
                impl_->pub_dwa_path.publish(dwa_path);
            }
            else if(impl_->current_mission == 1 || impl_->current_mission == 4 || impl_->current_mission == 5) {
                impl_->cmd = impl_->cut_in.Stanley(impl_->pose, impl_->ogm, target, impl_->curr_vel, impl_->current_mission);
            }
            else if (impl_->current_mission == 3) {
                impl_->cmd = impl_->lane_tracker.Stanley(impl_->lane, impl_->curr_vel);
            }
            else impl_->cmd = impl_->gps_tracker.Stanley(impl_->pose, target, impl_->curr_vel, is_slow);
        }
        else impl_->cmd = impl_->gps_tracker.Stanley(impl_->pose, target, impl_->curr_vel, is_slow);

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

    void Planner::CollisionCallback(const morai_msgs::CollisionData &msg)
    {   
        if(impl_->curr_vel < 1.0 && !msg.collision_object.empty() && msg.collision_object[0].type != 1) impl_->collision = true;
        else impl_->collision = false;
    }

    void Planner::laneCallback(const mm_common::lane_info &msg)
    {   
        impl_->lane = msg;
    }


    void Planner::trafficCallback(const std_msgs::Int32 &msg)
    {   
        impl_->traffic = msg.data;
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
