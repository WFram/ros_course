#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <action_turtle_commands/execute_turtle_commandsAction.h>

#include <vector>
#include <memory>


typedef action_turtle_commands::execute_turtle_commandsGoal ActionGoal;

int main(int argc, char **argv) {
    ros::init(argc, argv, "action_turtle_client");

    actionlib::SimpleActionClient<action_turtle_commands::execute_turtle_commandsAction>
        ac("action_turtle_server", true);

    ROS_INFO("Waiting for action server to start.");

    ac.waitForServer();

    ROS_INFO("Action server started, sending a goal.");

    std::vector<ActionGoal> vGoal;

    std::shared_ptr<ActionGoal> pGoal(new ActionGoal);
    pGoal->command = "forward";
    pGoal->s = 2;
    pGoal->angle = 0;
    vGoal.emplace_back(*(pGoal.get()));

    pGoal->command = "turn_right";
    pGoal->s = 0;
    pGoal->angle = -90;
    vGoal.emplace_back(*(pGoal.get()));

    pGoal->command = "forward";
    pGoal->s = 1;
    pGoal->angle = 0;
    vGoal.emplace_back(*(pGoal.get()));

    for (auto it = begin(vGoal); it != end(vGoal); ++it ) {
        ac.sendGoal(*it);
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
        // Finishing and saying "Action finished" means GETTING A RESULT 

        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
            finished_before_timeout = false;
        }
        else {
            ROS_INFO("Action did not finish before the time out.");
            break;
        }
    }

    return 0;
}