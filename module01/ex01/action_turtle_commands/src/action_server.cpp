#include "action_server_class.h"

Execute_turtle_commands::Execute_turtle_commands(std::string name) :
    as_(nh_, name, boost::bind(&Execute_turtle_commands::executeCB, this, _1), false),
    action_name_(name) {
        as_.start();
        pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
        sub_ = nh_.subscribe<turtlesim::Pose>("/turtle1/pose", 1000,
            boost::bind(&Execute_turtle_commands::mPoseCb, this, _1));
}
        
void Execute_turtle_commands::mPoseCb(const turtlesim::PoseConstPtr& pose) {
    if (mReached || mFistTime) {
        x0 = pose.get()->x;
        y0 = pose.get()->y;
        theta0 = pose.get()->theta;
        mFistTime = false;
        if (!mReached) {
            xs = x0;
            ys = y0;
        }
        mReached = false;
    }
    relDist = std::sqrt(std::pow(pose.get()->x - x0, 2) + std::pow(pose.get()->y - y0, 2));
    relAngle = std::sqrt(std::pow(pose.get()->theta - theta0, 2));
    mDist = std::fabs(pose.get()->x - xs) + std::fabs(pose.get()->y - ys);
    // ROS_INFO_STREAM("Relative distance: " << relDist);
    // ROS_INFO_STREAM("Relative angle: " << relAngle);
    // ROS_INFO_STREAM("Feedback: " << mDist);
}

void Execute_turtle_commands::executeCB(const action_turtle_commands::execute_turtle_commandsGoalConstPtr &goal) { // TODO Ptr
    ros::Rate r(1);

    ROS_INFO_STREAM(action_name_.c_str() << ": Executing, handling a command " <<
        goal->command);

    if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO_STREAM(action_name_.c_str() << " : Preempted");
        as_.setPreempted();
        mSuccess = false;
    }

    geometry_msgs::Twist twist;
    if (goal->command == "forward") {
        twist.linear.x = goal->s;
        ROS_INFO_STREAM("Generating control " << twist.linear.x << " m/s");
        // twist.linear.x = goal->s - relDist; // TODO: Make more flexible
    }
    else if (goal->command == "turn_left") {
        // twist.angular.z = goal->angle - relAngle;
        twist.angular.z = goal->angle * (M_PI / 180);
        ROS_INFO_STREAM("Generating control " << twist.angular.z << " rad/s");
    }
    else if (goal->command == "turn_right") {
        // twist.angular.z = -goal->angle + relAngle;
        twist.angular.z = goal->angle * (M_PI / 180);
        ROS_INFO_STREAM("Generating control " << twist.angular.z << " rad/s");
    }
    
    pub_.publish(twist);
    
    feedback_.odom = mDist;
    as_.publishFeedback(feedback_);
    ROS_INFO_STREAM("Feedback (Manhattan distance): " << mDist);
    
    if (relDist >= goal->s && relAngle >= goal->angle * (M_PI / 180)) {
        mReached = true;
    }

    if (mSuccess) {
        result_.result = true;
        ROS_INFO_STREAM(action_name_.c_str() << " Succeed!");
        as_.setSucceeded(result_);
    }

    r.sleep();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "action_turtle_server");

    Execute_turtle_commands exec("action_turtle_server");
    ros::spin();
}