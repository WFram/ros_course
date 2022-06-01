#include "ros/ros.h"
#include <std_msgs/UInt16.h>

int main (int argc, char** argv) {
    ros::init(argc, argv, "battery_status");

    ros::NodeHandle nh;

    ros::Rate r(10);

    ros::Publisher pub = nh.advertise<std_msgs::UInt16>("/battery_status", 1);
    std_msgs::UInt16 msg;
    float start_time = ros::Time::now().toSec();
    
    while (ros::ok()) {
        msg.data = (uint16_t) std::pow(ros::Time::now().toSec() - start_time, 2) - 1000;

        pub.publish(msg);
        r.sleep();
    }

    return 0;
}