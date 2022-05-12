#include "ros/ros.h"
#include "service_full_name/summ_full_name.h"
#include <cstdlib>

int main (int argc, char **argv) {

    ros::init(argc, argv, "client_name");

    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<service_full_name::summ_full_name>("summ_full_name");

    if (argc != 4) {
        ROS_INFO("Usage: client_name last_name name first_name)");
        return 1;
    }

    service_full_name::summ_full_name srv;
    srv.request.last_name = std::string(argv[1]);
    srv.request.name = std::string(argv[2]);
    srv.request.first_name = std::string(argv[3]);

    if (client.call(srv)) {
        ROS_INFO_STREAM("Concatenation: " << srv.response.full_name);
    }
    else {
        ROS_ERROR("Failed to call service summ_full_name");
        return 1;
    }

    return 0;
}