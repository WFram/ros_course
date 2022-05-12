#include "ros/ros.h"
#include "service_full_name/summ_full_name.h"

#include <string>

bool cb (service_full_name::summ_full_name::Request &req,
            service_full_name::summ_full_name::Response &res);

int main (int argc, char **argv) {
    
    ros::init(argc, argv, "service_name");

    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("summ_full_name", cb);
    ROS_INFO_STREAM("Ready to add two strings");
    ros::spin();

    return 0;
}

bool cb (service_full_name::summ_full_name::Request &req,
            service_full_name::summ_full_name::Response &res) {
    
    res.full_name = req.last_name + req.name + req.first_name;
    ROS_INFO_STREAM("Request: last_name = " << req.last_name <<
                        ", name = " << req.name <<
                        ", first_name = " << req.first_name);
    ROS_INFO_STREAM("Sending back response: " << res.full_name);
    
    return true;
}