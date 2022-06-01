#include "dummy_nodes.h"

#include <thread>
#include <chrono>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tree");

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<DummyNodes::Subscriber>("Subscriber");
    factory.registerNodeType<DummyNodes::Publisher>("Publisher");

    auto tree = factory.createTreeFromFile("/home/wfram/catkin_ws/src/robot_behavior/bt/tree.xml");

    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    while (ros::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.rootNode()->executeTick();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}