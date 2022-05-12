#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <action_turtle_commands/execute_turtle_commandsAction.h>
// #include <action_turtle_commands/execute_turtle_commandsGoal.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define M_PI 3.14

class Execute_turtle_commands {
public:
    Execute_turtle_commands(std::string name);
        
    ~Execute_turtle_commands(void) {}

        // TODO:
        // 1. Feedback (current distance)
        // 2. Relative distance
        // 3. Relative angle

    void mPoseCb(const turtlesim::PoseConstPtr& pose);

    void executeCB(const action_turtle_commands::execute_turtle_commandsGoalConstPtr &goal);

protected:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    actionlib::SimpleActionServer<action_turtle_commands::execute_turtle_commandsAction> as_; // TODO
    std::string action_name_;
    action_turtle_commands::execute_turtle_commandsFeedback feedback_;
    action_turtle_commands::execute_turtle_commandsResult result_;
    float x0, y0, theta0, xs = 0, ys = 0;
    bool mSuccess = true; // TODO Really?
    float relDist = 0.0, relAngle, mDist;
    bool mFistTime = true, mReached = false;
};