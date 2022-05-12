#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <iostream>
#include <memory>

#define PI 3.1415926

class MoveToGoal {
public:
    MoveToGoal::MoveToGoal();

    void poseCbk(const turtlesim::PoseConstPtr& msg);
    void moveGoal();
    void setGoal(double x, double y, double theta);

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    turtlesim::Pose curPose;

    double xdes, ydes, thetades;

    double xcur, ycur, thetacur;

    double toleranceX = 0.01, toleranceY = 0.01, toleranceTheta = 0.05;

    double Kv = 0.5, Ktheta = 1;
};