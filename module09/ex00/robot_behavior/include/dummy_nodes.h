#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>

namespace DummyNodes {

    class Subscriber : public BT::SyncActionNode {
    public:
        Subscriber(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config), r_(10) {
                sub_ = nh_.subscribe<std_msgs::UInt16>("/repeat_times", 1000,
                    boost::bind(&Subscriber::cb_, this, _1));
            }

        static BT::PortsList providedPorts() {
            return { BT::OutputPort<uint16_t>("sub_port") };
        }

        BT::NodeStatus tick() override {

            while (times_ < 1) {
                ros::spinOnce();
                r_.sleep();
            }

            setOutput("sub_port", times_);
            ROS_INFO("Subscriber SUCCESSED");
            return BT::NodeStatus::SUCCESS;

        }

        void cb_(const std_msgs::UInt16ConstPtr &msg) {
            times_ = msg->data;
            ROS_INFO_STREAM("Times: " << times_);
        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Rate r_;
        uint16_t times_ = 0;
    };

    class Publisher : public BT::SyncActionNode {
    public:
        Publisher(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config) {
                pub_ = nh_.advertise<std_msgs::String>("/hello_message", 1000);
            }

        static BT::PortsList providedPorts() {
            return { BT::InputPort<uint16_t>("pub_port") };
        }
        
        BT::NodeStatus tick() override {
            ros::Duration(1).sleep();
            BT::Optional<uint16_t> msg = getInput<uint16_t>("pub_port");
            if (!msg) {
                throw BT::RuntimeError("missing required input [message]: ",
                                        msg.error());
            }

            uint16_t times = msg.value();
            std_msgs::String string_msg;
            string_msg.data = "Hello ROS!"; 

            for (uint16_t i = 0; i < times; i++) {
                pub_.publish(string_msg);
            }

            return BT::NodeStatus::SUCCESS;
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
    };

}