#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>

#include <thread>
#include <chrono>

struct Pose2D {
    double x, y, theta;
};

namespace DummyNodes {

    class MoveBaseAction1 : public BT::AsyncActionNode {
    public:
        MoveBaseAction1(const std::string &name)
            : BT::AsyncActionNode(name, {}) {
                ros::param::get("first_goal", wps_);
                goal_.x = wps_[0];
                goal_.y = wps_[1];
                goal_.theta = wps_[2];
            }

        BT::NodeStatus tick() override {
            ROS_INFO("Moving to %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            ROS_INFO("Successfully reached %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
            return BT::NodeStatus::SUCCESS;
        }
        
    private:
        ros::NodeHandle nh_;
        Pose2D goal_;
        std::vector<double> wps_;
    };

    class MoveBaseAction2 : public BT::AsyncActionNode {
    public:
        MoveBaseAction2(const std::string &name)
            : BT::AsyncActionNode(name, {}) {
                ros::param::get("second_goal", wps_);
                goal_.x = wps_[0];
                goal_.y = wps_[1];
                goal_.theta = wps_[2];
            }

        BT::NodeStatus tick() override {
            ROS_INFO("Moving to %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            ROS_INFO("Successfully reached %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
            return BT::NodeStatus::SUCCESS;
        }
        
    private:
        ros::NodeHandle nh_;
        Pose2D goal_;
        std::vector<double> wps_;
    };

    class MoveBaseAction3 : public BT::AsyncActionNode {
    public:
        MoveBaseAction3(const std::string &name)
            : BT::AsyncActionNode(name, {}) {
                ros::param::get("third_goal", wps_);
                goal_.x = wps_[0];
                goal_.y = wps_[1];
                goal_.theta = wps_[2];
            }

        BT::NodeStatus tick() override {
            ROS_INFO("Moving to %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            ROS_INFO("Successfully reached %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
            return BT::NodeStatus::SUCCESS;
        }
        
    private:
        ros::NodeHandle nh_;
        Pose2D goal_;
        std::vector<double> wps_;
    };

}