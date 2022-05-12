#include <move_to_goal/move_to_goal.h>

void move(ros::Publisher& pub, double vel, double distance, bool isForward);
void rotate(ros::Publisher& pub, double vel, double angle, bool clockWise);

MoveToGoal::MoveToGoal() {
    pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    sub = nh.subscribe<turtlesim::Pose>("turtle1/pose", 10,
        boost::bind(&MoveToGoal::poseCbk, this, _1));
}

void MoveToGoal::setGoal(double x, double y, double theta) {
    xdes = x;
    ydes = y;
    thetades = theta;
}

void MoveToGoal::poseCbk(const turtlesim::PoseConstPtr& msg) {
    xcur = msg.get()->x;
    ycur = msg.get()->y;
    thetacur = msg.get()->theta;
}

void MoveToGoal::moveGoal() {
    std::shared_ptr<geometry_msgs::Twist> twist(new geometry_msgs::Twist);
    while (std::abs(xdes - xcur) >= toleranceX &&
            std::abs(ydes - ycur) >= toleranceY) {
        ROS_INFO_STREAM("Cur Pose X: " << xdes);
        ROS_INFO_STREAM("Cur Pose Y: " << ydes << std::endl);
        twist->linear.x =
            Kv * std::sqrt(std::pow(xdes - xcur, 2) + std::pow(ydes - ycur, 2));
        twist->angular.z = std::atan2(ydes - ycur, xdes - xcur); // TODO; Check whether radians or not
        ros::Rate r(100);
        pub.publish(*twist);
        r.sleep();
    }

    twist->linear.x = 0.0;
    twist->angular.z = 0.0;
    pub.publish(*twist);

    while (std::abs(thetades - thetacur) >= toleranceTheta) {
        twist->angular.z = Ktheta * (thetades - thetacur);
        ros::Rate r(100);
        pub.publish(*twist);
        r.sleep();
    }
    
    twist->angular.z = 0.0;
    pub.publish(*twist);
    ROS_INFO_STREAM("Goal reached!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_to_goal");

    if (argc != 4) {
        ROS_ERROR_STREAM("Use: rosrun move_to_goal move_to_goal x y theta");
        return 1;
    }

    MoveToGoal moveToGoal;

    ROS_INFO_STREAM("********** WELCOME! **********" << std::endl);

    moveToGoal.setGoal(atof(argv[1]), atof(argv[2]), atof(argv[3]) * (PI / 180));
    moveToGoal.moveGoal();

    // while (ros::ok)
    ros::spin();
    return 0;
}

void move(ros::Publisher& pub, double vel, double distance, bool isForward) {
    geometry_msgs::Twist twist;
    if (isForward)
        twist.linear.x = std::abs(vel);
    else
        twist.linear.x = -std::abs(vel);

    double curDistance = 0.0;
    double t0 = ros::Time::now().toSec();
    ros::Rate r(10);

    while (curDistance < distance) {
        pub.publish(twist);
        double t1 = ros::Time::now().toSec();
        curDistance = vel * (t1 - t0);
        ros::spinOnce();
        r.sleep();
    }
    twist.linear.x = 0.0;
    pub.publish(twist);
}

void rotate(ros::Publisher& pub, double vel, double angle, bool clockWise) {
    geometry_msgs::Twist twist;
    if (clockWise)
        twist.angular.z = -std::abs(vel);
    else
        twist.angular.z = std::abs(vel);

    double relAngle = 0.0;
    double t0 = ros::Time::now().toSec();
    ros::Rate r(10);

    while (relAngle < angle) {
        pub.publish(twist);
        double t1 = ros::Time::now().toSec();
        relAngle = vel * (t1 - t0);
        ros::spinOnce();
        r.sleep();
    }
    twist.angular.z = 0.0;
    pub.publish(twist);
}