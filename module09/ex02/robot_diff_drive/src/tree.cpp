#include "dummy_nodes.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "tree");

    int times_to_repeat;
    std::string tree_file;
    ros::param::get("times_to_repeat", times_to_repeat);
    ros::param::get("tree_file", tree_file);

    BT::BehaviorTreeFactory factory;

    // Or try to give the parameters to the ports directly from launch file

    // Create a nodehandle here
    // Read a parameter from the launch file
    // Fill the structure with point data
    // 

    factory.registerNodeType<DummyNodes::MoveBaseAction1>("MoveBaseAction1");
    factory.registerNodeType<DummyNodes::MoveBaseAction2>("MoveBaseAction2");
    factory.registerNodeType<DummyNodes::MoveBaseAction3>("MoveBaseAction3");

    auto tree = factory.createTreeFromFile(tree_file);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    ROS_INFO("Times: %d", times_to_repeat);

    while (ros::ok() && status == BT::NodeStatus::RUNNING) {
        if (times_to_repeat < 0) {
            while (1) { 
                status = tree.rootNode()->executeTick();
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        else {
            for (int i = 0; i < times_to_repeat; i++) {
                status = tree.rootNode()->executeTick();
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
    }

    return 0;
}