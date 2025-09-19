#include "mm_pose_parser/pose_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <GeographicLib/UTMUPS.hpp>

namespace pose_parser
{
    struct PoseParser::Impl
    {
        std::string start;
        ros::Subscriber sub_gps;
        ros::Subscriber sub_heading;
        ros::Subscriber sub_mag;
        ros::Publisher pub_state;

        double origin_lat; //[lat,lon]
        double origin_lon;
        float origin_orientation = 0.0;
        
        float heading;
        float mag_heading;
        bool got_heading = false;
        bool got_mag = false;
        bool origin_orientation_initialized = false;
        bool carla_simulator = false;
    };

    PoseParser::PoseParser() : impl_(new Impl) {}
    PoseParser::~PoseParser() {}

    void PoseParser::Init(ros::NodeHandle &nh)
    {
        ROS_ASSERT(nh.getParam("start", impl_->start));
        ROS_ASSERT(nh.getParam("carla_simulator", impl_->carla_simulator));
        ReadOriginFromCSV(impl_->start);

        if(impl_->carla_simulator) 
        {
            impl_->origin_orientation_initialized = true;
        }
        
        // initialize subscriber
        impl_->sub_gps = nh.subscribe("gps", 2,  &PoseParser::GPSCallback, this);
        impl_->sub_heading = nh.subscribe("heading", 2,  &PoseParser::HeadingCallback, this);
        if(!impl_->carla_simulator) impl_->sub_mag = nh.subscribe("/imu/mag", 2,  &PoseParser::MagCallback, this);
        // initialize publishers
        impl_->pub_state = nh.advertise<geometry_msgs::Pose2D>("state", 1);
    }

    void PoseParser::SetOrigin(double lat, double lon)
    {
        impl_->origin_lat = lat;
        impl_->origin_lon = lon;
    }

    void PoseParser::ReadOriginFromCSV(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Failed to open origin CSV file: " << filename << std::endl;
            return;
        }

        std::string line;
        if (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string item;

            std::getline(ss, item, ',');
            double lat = std::stod(item);
            std::getline(ss, item, ',');
            double lon = std::stod(item);

            SetOrigin(lat, lon);
            ROS_INFO_STREAM("Origin loaded from " << filename << ": (" << lat << ", " << lon << ")");
        }
        else
        {
            std::cerr << "Origin CSV file is empty: " << filename << std::endl;
        }
    }

    void PoseParser::HeadingCallback(const sensor_msgs::Imu &msg)
    {
        impl_->heading = CalHeading(msg.orientation);
        impl_->got_heading = true;
    }

    void PoseParser::MagCallback(const geometry_msgs::Vector3Stamped &msg)
    {
        impl_->mag_heading = atan2(msg.vector.y,msg.vector.x)*180/M_PI;
        impl_->got_mag = true;

        SetOriginOrientation();
    }

    void PoseParser::GPSCallback(const morai_msgs::GPSMessage &msg)
    {
        double lat = msg.latitude;
        double lon = msg.longitude;
        double h = msg.altitude;
        double x0 = msg.eastOffset;
        double y0 = msg.northOffset;

        if (!impl_->origin_orientation_initialized)
            return;

        geometry_msgs::Pose2D pose = ConvertGps2XYYaw(lon,lat, h, x0, y0, impl_->heading);
        
        impl_->pub_state.publish(pose);
    }

    geometry_msgs::Pose2D PoseParser::ConvertGps2XYYaw(double lon, double lat, double h, double x0, double y0, float heading)
    {
        geometry_msgs::Pose2D temp;
        // converting current position coordinate gps to xy
        using namespace GeographicLib;

        int zone;
        bool northp;
        double x, y;
        UTMUPS::Forward(lat, lon, zone, northp, x, y);

        // Compute ENU coordinates relative to reference point
        temp.x  = x - x0;
        temp.y = y - y0;
        temp.theta = heading;

        return temp;
    }

    float PoseParser::CalHeading(const geometry_msgs::Quaternion &msg)
    {
        tf::Quaternion q(
            msg.x,
            msg.y,
            msg.z,
            msg.w
        );

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        float yaw_deg = yaw * 180.0 / M_PI;
        yaw_deg = yaw_deg + impl_->origin_orientation;

        // Wrap to [-180, 180]
        while (yaw_deg > 180.0)
            yaw_deg -= 360.0;
        while (yaw_deg < -180.0)
            yaw_deg += 360.0;
        
        return yaw_deg;
    }

    void PoseParser::SetOriginOrientation()
    {
        if (!impl_->got_heading || !impl_->got_mag || impl_->origin_orientation_initialized) {
            return;
        }
            
        impl_->origin_orientation = 90.0 - impl_->mag_heading - impl_->heading;

        // Normalize to [-180, 180]
        if (impl_->origin_orientation > 180.0)
            impl_->origin_orientation -= 360.0;
        else if (impl_->origin_orientation < -180.0)
            impl_->origin_orientation += 360.0;

        impl_->origin_orientation_initialized = true;

        ROS_INFO_STREAM("Computed origin orientation: " << impl_->origin_orientation << " degrees");
    }
}