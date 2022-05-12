#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <sstream>

int main(int argc, char **argv) {

    ros::init(argc, argv, "publisher");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("cmd_text", 1000);

    // ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        // loop_rate.sleep();
    }

    return 0;
}
