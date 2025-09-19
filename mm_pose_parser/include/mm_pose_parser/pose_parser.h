#ifndef MM_GPS_MANAGER_MM_GPS_MANAGER_GPS_MANAGER_H_
#define MM_GPS_MANAGER_MM_GPS_MANAGER_GPS_MANAGER_H_

#include <morai_msgs/GPSMessage.h>
#include <ublox_msgs/NavPVT.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include <ros/ros.h>
#include <memory>


namespace pose_parser
{
    class PoseParser
    {
    public:
        PoseParser();
        ~PoseParser();

        void Init(ros::NodeHandle &nh);
        void SetOrigin(double lat, double lon);
        void ReadOriginFromCSV(const std::string &filename);
        void GPSCallback(const morai_msgs::GPSMessage &msg);
        void HeadingCallback(const sensor_msgs::Imu &msg);
        void MagCallback(const geometry_msgs::Vector3Stamped &msg);
        void Run();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
        geometry_msgs::Pose2D ConvertGps2XYYaw(double lon, double lat, double h, double x0, double y0, float heading);
        float CalHeading(const geometry_msgs::Quaternion &msg);
        void SetOriginOrientation();
    };
}

#endif // RR_GPS_MANAGER_RR_GPS_MANAGER_GPS_MANAGER_H_