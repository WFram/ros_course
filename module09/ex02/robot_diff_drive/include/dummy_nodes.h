#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalID.h>

#include <thread>
#include <chrono>

struct Pose2D {
    double x, y, theta;
};

namespace DummyNodes {

    class MoveBaseAction1 : public BT::SyncActionNode {
    public:
        MoveBaseAction1(const std::string &name)
            : BT::SyncActionNode(name, {}), r_(5) {
                ros::param::get("first_goal", wps_);
                ros::param::get("dock_station_pose", dock_station_wps_);
                nh_.getParam("battery_threshold", battery_threshold_);
                nh_.setCallbackQueue(&cb_queue_);
                pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
                sub_status_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1,
                    boost::bind(&MoveBaseAction1::cbGoalStatus_, this, _1));
                sub_charge_ = nh_.subscribe<std_msgs::UInt16>("/battery_status", 1,
                    boost::bind(&MoveBaseAction1::cbBattery_, this, _1));

                goal_.x = wps_[0];
                goal_.y = wps_[1];
                goal_.theta = wps_[2];

                dock_station_pose_.x = dock_station_wps_[0];
                dock_station_pose_.y = dock_station_wps_[1];
                dock_station_pose_.theta = dock_station_wps_[2];
            }

        BT::NodeStatus tick() override {
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";
            msg.pose.position.x = goal_.x;
            msg.pose.position.y = goal_.y;
            msg.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, goal_.theta);
            msg.pose.orientation.x = q.x();
            msg.pose.orientation.y = q.y();
            msg.pose.orientation.z = q.z();
            msg.pose.orientation.w = q.w();

            ROS_INFO("Moving to %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            pub_.publish(msg);

            ros::AsyncSpinner spinner(0, &cb_queue_);

            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            while (!is_succeed_) {
                spinner.start();

                if (need_charge_) {
                    geometry_msgs::PoseStamped g_msg;
                    g_msg.header.stamp = ros::Time::now();
                    g_msg.header.frame_id = "map";
                    g_msg.pose.position.x = dock_station_pose_.x;
                    g_msg.pose.position.y = dock_station_pose_.y;
                    g_msg.pose.position.z = 0.0;

                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, dock_station_pose_.theta);
                    g_msg.pose.orientation.x = q.x();
                    g_msg.pose.orientation.y = q.y();
                    g_msg.pose.orientation.z = q.z();
                    g_msg.pose.orientation.w = q.w();

                    ROS_INFO("Moving to the dock station %0.8f, %0.8f, %0.8f",
                            dock_station_pose_.x,
                            dock_station_pose_.y,
                            dock_station_pose_.theta);
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    pub_.publish(g_msg);
    
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    while (!is_succeed_) {
                        spinner.start();
                        r_.sleep();
                    }

                    spinner.stop();
                    ROS_INFO("Dock reached %0.8f, %0.8f, %0.8f",
                            dock_station_pose_.x,
                            dock_station_pose_.y,
                            dock_station_pose_.theta);
                    
                    while (need_charge_) {
                        spinner.start(); // Here it should spin forever (till reaching 100%)
                        // How many times it will spin?
                        ROS_INFO("Charging...");
                    }
                    spinner.stop(); // It might be a conflict here
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    ROS_INFO("Moving to %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
                    pub_.publish(msg);
                    is_succeed_ = false;
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                }
                r_.sleep();
            }

            spinner.stop();
            is_succeed_ = false;
            ROS_INFO("Successfully reached %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
            return BT::NodeStatus::SUCCESS;
        }

        void cbGoalStatus_(const actionlib_msgs::GoalStatusArrayConstPtr &msg) {
            if (msg->status_list.empty()) {
                ROS_WARN("Queue is empty");
                return;
            }
            
            if (msg->status_list.back().status == actionlib_msgs::GoalStatus::SUCCEEDED) {
                ROS_INFO("Goal 1. Status: %d", msg->status_list.back().status);
                is_succeed_ = true;
            }
        }

        void cbBattery_(const std_msgs::UInt16ConstPtr &msg) {
            ROS_INFO("Current battery charge: %d", msg->data);
            if (msg->data <= battery_threshold_)
                need_charge_ = true;
            else
                need_charge_ = false;
        }
        
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::Subscriber sub_status_;
        ros::Subscriber sub_charge_;
        ros::CallbackQueue cb_queue_;
        ros::Rate r_;
        bool is_succeed_ = false;
        bool need_charge_ = false;
        actionlib_msgs::GoalID last_id_;
        Pose2D goal_, dock_station_pose_;
        std::vector<double> wps_;
        std::vector<double> dock_station_wps_;
        int battery_threshold_;
    };

    class MoveBaseAction2 : public BT::SyncActionNode {
    public:
        MoveBaseAction2(const std::string &name)
            : BT::SyncActionNode(name, {}), r_(5) {
                ros::param::get("second_goal", wps_);
                ros::param::get("dock_station_pose", dock_station_wps_);
                nh_.getParam("battery_threshold", battery_threshold_);
                nh_.setCallbackQueue(&cb_queue_);
                pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
                sub_status_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1,
                    boost::bind(&MoveBaseAction2::cbGoalStatus_, this, _1));
                sub_charge_ = nh_.subscribe<std_msgs::UInt16>("/battery_status", 1,
                    boost::bind(&MoveBaseAction2::cbBattery_, this, _1));

                goal_.x = wps_[0];
                goal_.y = wps_[1];
                goal_.theta = wps_[2];

                dock_station_pose_.x = dock_station_wps_[0];
                dock_station_pose_.y = dock_station_wps_[1];
                dock_station_pose_.theta = dock_station_wps_[2];
            }

        BT::NodeStatus tick() override {
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";
            msg.pose.position.x = goal_.x;
            msg.pose.position.y = goal_.y;
            msg.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, goal_.theta);
            msg.pose.orientation.x = q.x();
            msg.pose.orientation.y = q.y();
            msg.pose.orientation.z = q.z();
            msg.pose.orientation.w = q.w();

            ROS_INFO("Moving to %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            pub_.publish(msg);

            ros::AsyncSpinner spinner(0, &cb_queue_);

            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            while (!is_succeed_) {
                spinner.start();

                if (need_charge_) {
                    geometry_msgs::PoseStamped g_msg;
                    g_msg.header.stamp = ros::Time::now();
                    g_msg.header.frame_id = "map";
                    g_msg.pose.position.x = dock_station_pose_.x;
                    g_msg.pose.position.y = dock_station_pose_.y;
                    g_msg.pose.position.z = 0.0;

                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, dock_station_pose_.theta);
                    g_msg.pose.orientation.x = q.x();
                    g_msg.pose.orientation.y = q.y();
                    g_msg.pose.orientation.z = q.z();
                    g_msg.pose.orientation.w = q.w();

                    ROS_INFO("Moving to the dock station %0.8f, %0.8f, %0.8f",
                            dock_station_pose_.x,
                            dock_station_pose_.y,
                            dock_station_pose_.theta);
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    pub_.publish(g_msg);
    
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    while (!is_succeed_) {
                        spinner.start();
                        r_.sleep();
                    }

                    spinner.stop();
                    ROS_INFO("Dock reached %0.8f, %0.8f, %0.8f",
                            dock_station_pose_.x,
                            dock_station_pose_.y,
                            dock_station_pose_.theta);
                    
                    while (need_charge_) {
                        spinner.start(); // Here it should spin forever (till reaching 100%)
                        // How many times it will spin?
                        ROS_INFO("Charging...");
                    }
                    spinner.stop(); // It might be a conflict here
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    ROS_INFO("Moving to %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
                    pub_.publish(msg);
                    is_succeed_ = false;
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                }
                r_.sleep();
            }

            spinner.stop();
            is_succeed_ = false;
            ROS_INFO("Successfully reached %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
            return BT::NodeStatus::SUCCESS;
        }

        void cbGoalStatus_(const actionlib_msgs::GoalStatusArrayConstPtr &msg) {
            if (msg->status_list.empty()) {
                ROS_WARN("Queue is empty");
                return;
            }
            
            if (msg->status_list.back().status == actionlib_msgs::GoalStatus::SUCCEEDED) {
                ROS_INFO("Goal 2. Status: %d", msg->status_list.back().status);
                is_succeed_ = true;
            }
        }

        void cbBattery_(const std_msgs::UInt16ConstPtr &msg) {
            ROS_INFO("Current battery charge: %d", msg->data);
            if (msg->data <= battery_threshold_)
                need_charge_ = true;
            else
                need_charge_ = false;
        }
        
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::Subscriber sub_status_;
        ros::Subscriber sub_charge_;
        ros::CallbackQueue cb_queue_;
        ros::Rate r_;
        bool is_succeed_ = false;
        bool need_charge_ = false;
        actionlib_msgs::GoalID last_id_;
        Pose2D goal_, dock_station_pose_;
        std::vector<double> wps_;
        std::vector<double> dock_station_wps_;
        int battery_threshold_;
    };

    class MoveBaseAction3 : public BT::SyncActionNode {
    public:
        MoveBaseAction3(const std::string &name)
            : BT::SyncActionNode(name, {}), r_(5) {
                ros::param::get("third_goal", wps_);
                ros::param::get("dock_station_pose", dock_station_wps_);
                nh_.getParam("battery_threshold", battery_threshold_);
                nh_.setCallbackQueue(&cb_queue_);
                pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
                sub_status_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1,
                    boost::bind(&MoveBaseAction3::cbGoalStatus_, this, _1));
                sub_charge_ = nh_.subscribe<std_msgs::UInt16>("/battery_status", 1,
                    boost::bind(&MoveBaseAction3::cbBattery_, this, _1));

                goal_.x = wps_[0];
                goal_.y = wps_[1];
                goal_.theta = wps_[2];

                dock_station_pose_.x = dock_station_wps_[0];
                dock_station_pose_.y = dock_station_wps_[1];
                dock_station_pose_.theta = dock_station_wps_[2];
            }

        BT::NodeStatus tick() override {
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";
            msg.pose.position.x = goal_.x;
            msg.pose.position.y = goal_.y;
            msg.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, goal_.theta);
            msg.pose.orientation.x = q.x();
            msg.pose.orientation.y = q.y();
            msg.pose.orientation.z = q.z();
            msg.pose.orientation.w = q.w();

            ROS_INFO("Moving to %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            pub_.publish(msg);

            ros::AsyncSpinner spinner(0, &cb_queue_);

            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            while (!is_succeed_) {
                spinner.start();

                if (need_charge_) {
                    geometry_msgs::PoseStamped g_msg;
                    g_msg.header.stamp = ros::Time::now();
                    g_msg.header.frame_id = "map";
                    g_msg.pose.position.x = dock_station_pose_.x;
                    g_msg.pose.position.y = dock_station_pose_.y;
                    g_msg.pose.position.z = 0.0;

                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, dock_station_pose_.theta);
                    g_msg.pose.orientation.x = q.x();
                    g_msg.pose.orientation.y = q.y();
                    g_msg.pose.orientation.z = q.z();
                    g_msg.pose.orientation.w = q.w();

                    ROS_INFO("Moving to the dock station %0.8f, %0.8f, %0.8f",
                            dock_station_pose_.x,
                            dock_station_pose_.y,
                            dock_station_pose_.theta);
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    pub_.publish(g_msg);
    
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    while (!is_succeed_) {
                        spinner.start();
                        r_.sleep();
                    }

                    spinner.stop();
                    ROS_INFO("Dock reached %0.8f, %0.8f, %0.8f",
                            dock_station_pose_.x,
                            dock_station_pose_.y,
                            dock_station_pose_.theta);
                    
                    while (need_charge_) {
                        spinner.start(); // Here it should spin forever (till reaching 100%)
                        // How many times it will spin?
                        ROS_INFO("Charging...");
                    }
                    spinner.stop(); // It might be a conflict here
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    ROS_INFO("Moving to %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
                    pub_.publish(msg);
                    is_succeed_ = false;
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                }
                r_.sleep();
            }

            spinner.stop();
            is_succeed_ = false;
            ROS_INFO("Successfully reached %0.8f, %0.8f, %0.8f", goal_.x, goal_.y, goal_.theta);
            return BT::NodeStatus::SUCCESS;
        }

        void cbGoalStatus_(const actionlib_msgs::GoalStatusArrayConstPtr &msg) {
            if (msg->status_list.empty()) {
                ROS_WARN("Queue is empty");
                return;
            }
            
            if (msg->status_list.back().status == actionlib_msgs::GoalStatus::SUCCEEDED) {
                ROS_INFO("Goal 3. Status: %d", msg->status_list.back().status);
                is_succeed_ = true;
            }
        }

        void cbBattery_(const std_msgs::UInt16ConstPtr &msg) {
            ROS_INFO("Current battery charge: %d", msg->data);
            if (msg->data <= battery_threshold_)
                need_charge_ = true;
            else
                need_charge_ = false;
        }
        
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::Subscriber sub_status_;
        ros::Subscriber sub_charge_;
        ros::CallbackQueue cb_queue_;
        ros::Rate r_;
        bool is_succeed_ = false;
        bool need_charge_ = false;
        actionlib_msgs::GoalID last_id_;
        Pose2D goal_, dock_station_pose_;
        std::vector<double> wps_;
        std::vector<double> dock_station_wps_;
        int battery_threshold_;
    };

}