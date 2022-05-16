#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

void scanCbk(const sensor_msgs::LaserScanConstPtr &msg,
                const ros::Publisher &pub) {
    geometry_msgs::Twist twist;

    twist.linear.x = 0.4;
    
    for (auto &it: msg->ranges) {
        if ((it - 0.11) * (it - 1.0) <= 0) {
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
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1000,
        boost::bind(scanCbk, _1, pub));

    ros::spin();

    return 0;
}