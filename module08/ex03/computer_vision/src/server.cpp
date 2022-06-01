#include <computer_vision/ImageConverter.h>

int main (int argc, char** argv) {
    ros::init(argc, argv, "server");

    ImageConverterAR ic;

    ros::spin();
    return 0;
}