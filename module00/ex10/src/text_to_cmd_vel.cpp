#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <string>
#include <sstream>

void cmdTextCb (const std_msgs::String::ConstPtr &msg,
                ros::Publisher &pub);

int main(int argc, char **argv) {

    ros::init(argc, argv, "text_to_cmd_vel");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

    ros::Subscriber sub = nh.subscribe<std_msgs::String>("cmd_text", 1000, boost::bind(&cmdTextCb, _1, pub));

    ros::spin();

    return 0;
}

void cmdTextCb (const std_msgs::String::ConstPtr &msg,
                ros::Publisher &pub) {

    geometry_msgs::Twist twist;
    
    std::string str;
    str = msg.get()->data.c_str();
    if (str == "turn_right")
        twist.angular.z = -1.5;
    if (str == "turn_left")
        twist.angular.z = 1.5;
    if (str == "move_forward")
        twist.linear.x = 1.0;
    if (str == "move_backward")
        twist.linear.x = -1.0;

    pub.publish(twist);
}