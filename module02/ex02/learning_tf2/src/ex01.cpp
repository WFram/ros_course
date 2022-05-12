#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>
#include <functional>
#include <memory>

void cbk(const turtlesim::PoseConstPtr &msg) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform;

    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "turtle1";
    transform.transform.translation.x = msg->x;
    transform.transform.translation.y = msg->y;
    transform.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    br.sendTransform(transform);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ex01");

    ros::NodeHandle nh;

    double radius, direction;
    nh.param<double>("radius", radius, 2.0);
    nh.param<double>("direction_of_rotation", direction, -1);
    radius = 2.0;
    direction = -1;

    ros::service::waitForService("spawn");
    ros::ServiceClient srv = nh.serviceClient<turtlesim::Spawn>("spawn");

    std::shared_ptr<turtlesim::Spawn> spawner(new turtlesim::Spawn);

    spawner->request.x = 1.0;
    spawner->request.y = 1.0;
    spawner->request.theta = 0.0;
    spawner->request.name = "turtle2";
    srv.call(*spawner);

    spawner->request.x = 0.0;
    spawner->request.y = 0.0;
    spawner->request.theta = 0.0;
    spawner->request.name = "turtle1";
    srv.call(*spawner);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("turtle1/pose", 10, &cbk);

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformCarrot;
    geometry_msgs::TransformStamped transformFollow;

    transformCarrot.header.frame_id = "turtle1";
    transformCarrot.child_frame_id = "carrot";

    transformCarrot.transform.translation.z = 0.0;
    transformCarrot.transform.rotation.x = 0.0;
    transformCarrot.transform.rotation.y = 0.0;
    transformCarrot.transform.rotation.z = 0.0;
    transformCarrot.transform.rotation.w = 1.0;

    geometry_msgs::Twist vel;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener list(tfBuffer);

    ros::Rate r(10);

    while(ros::ok()) {
        transformCarrot.transform.translation.x = direction * radius * std::sin(ros::Time::now().toSec());
        transformCarrot.transform.translation.y = direction * radius * std::cos(ros::Time::now().toSec());

        transformCarrot.header.stamp = ros::Time::now();
        br.sendTransform(transformCarrot);

        try {
            transformFollow = tfBuffer.lookupTransform("carrot", "turtle2", ros::Time::now()); // TODO: Check parent-child
        }
        catch (tf2::LookupException &ex) {
            ROS_WARN("%s", ex.what());
        }

        vel.angular.z = 4.0 * atan2(transformFollow.transform.translation.y,
                                    transformFollow.transform.translation.x);

        vel.linear.x = 0.5 * sqrt(pow(transformFollow.transform.translation.x, 2) +
                                    pow(transformFollow.transform.translation.y, 2));

        pub.publish(vel);

        // ROS

        // r.sleep();

        ros::spinOnce();
    }

    return 0;
}