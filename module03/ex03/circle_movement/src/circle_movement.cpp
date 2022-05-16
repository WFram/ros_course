#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_movement");

    ros::NodeHandle nh;

    ros::Rate r(50);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    geometry_msgs::Twist twist;

    while (nh.ok()) {
        twist.linear.x = 0.4;
        twist.angular.z = 0.5;

        pub.publish(twist);

        r.sleep();
    }
    
    return 0;
}