#include "ros/ros.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

std::string static_turtle_name;

int main(int argc, char **argv) {
    ros::init(argc, argv, "static_turtle_tf2_broadcaster");
    
    if (argc != 8) {
        ROS_ERROR_STREAM("Usage: static_turtle_tf2_broadcaster child_frame_name x y z r p y");
        return 1;
    }

    if (argv[1] == "world") {
        ROS_ERROR_STREAM("Static transform can't be world");
        return 1;
    }

    static_turtle_name = argv[1];
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = static_turtle_name;
    
    static_transformStamped.transform.translation.x = atof(argv[2]);
    static_transformStamped.transform.translation.y = atof(argv[3]);
    static_transformStamped.transform.translation.z = atof(argv[4]);

    tf2::Quaternion q;
    q.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
    static_transformStamped.transform.rotation.x = q.x();
    static_transformStamped.transform.rotation.y = q.y();
    static_transformStamped.transform.rotation.z = q.z();
    static_transformStamped.transform.rotation.w = q.w();
    static_broadcaster.sendTransform(static_transformStamped);

    ros::spin();
    return 0;
}