#include <computer_vision/ImageConverter.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "color_switcher_node");

    ImageConverterSwitch ic;

    ros::spin();

    return 0;
}