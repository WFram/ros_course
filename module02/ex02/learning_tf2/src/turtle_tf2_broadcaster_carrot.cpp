#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "turtle_tf2_broadcaster_carrot");
    
  ros::NodeHandle node;
  
  double radius, direction;
  node.param<double>("radius", radius, 2.0);
  node.param<double>("direction_of_rotation", direction, -1);
  ROS_WARN_STREAM("Dir: " << direction);
  ROS_WARN_STREAM("Radius: " << radius);

  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformCarrot;

  transformCarrot.header.frame_id = "/turtle1";
  transformCarrot.child_frame_id = "/carrot";

  transformCarrot.transform.translation.z = 0.0;
  transformCarrot.transform.rotation.x = 0.0;
  transformCarrot.transform.rotation.y = 0.0;
  transformCarrot.transform.rotation.z = 0.0;
  transformCarrot.transform.rotation.w = direction;
  ROS_INFO_STREAM("Quat: " << transformCarrot.transform.rotation.w);

  ros::Rate r(500);

  while (node.ok()) {
    transformCarrot.transform.translation.x = direction* radius * std::sin(ros::Time::now().toSec());
    transformCarrot.transform.translation.y = radius * std::cos(ros::Time::now().toSec());
    
    transformCarrot.header.stamp = ros::Time::now();
    br.sendTransform(transformCarrot);

    r.sleep();
  }

  return 0;
}