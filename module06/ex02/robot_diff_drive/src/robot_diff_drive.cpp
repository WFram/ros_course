#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

void scanCbk(const sensor_msgs::PointCloud2ConstPtr &msg,
                const ros::Publisher &pub) {
    geometry_msgs::Twist twist;

    twist.linear.x = 0.4;

    PointCloudXYZ cloud;
    pcl::fromROSMsg(*msg, cloud);

    for (auto &it: cloud.points) {
        float dist = std::sqrt(std::pow(it.x, 2) + std::pow(it.y, 2) + std::pow(it.z, 2));
        if ((dist - 0.51) * (dist - 1.0) <= 0) {
            twist.linear.x = 0.0;
            break;
        }
    }

    pub.publish(twist);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_diff_drive");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/kinect/depth/points", 1000,
        boost::bind(scanCbk, _1, pub));

    ros::spin();

    return 0;
}