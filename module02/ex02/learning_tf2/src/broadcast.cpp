#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <turtlesim/Pose.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_tf2_broadcaster");

    ros::NodeHandle node;

    tf2_ros::TransformBroadcaster br1, br2;
    geometry_msgs::TransformStamped transform1, transform2;

    transform.header.frame_id = "world";
    transform.child_frame_id = "turtle1";
    transform.transform.translation.x = 0.5;
    transform.transform.translation.y = 1.0;
    transform.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    ros::Rate r(100);

    while (ros::ok()) {
        transform1.header.stamp = ros::Time::now();
        transform2.header.stamp = ros::Time::now();
        br1.sendTransform(transform1);
        br2.sendTransform(transform2);
        r.sleep();
    }

    return 0;
}