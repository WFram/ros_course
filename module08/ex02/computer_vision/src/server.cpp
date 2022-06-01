#include <computer_vision/ImageConverter.h>

int main (int argc, char** argv) {
    ros::init(argc, argv, "server");

    ImageConverterCorrection ic;

    dynamic_reconfigure::Server<computer_vision::CannyDetectionConfig> server;
    dynamic_reconfigure::Server<computer_vision::CannyDetectionConfig>::CallbackType f;

    f = boost::bind(&ImageConverterCorrection::serverCb, &ic, _1, _2);

    server.setCallback(f);

    ros::spin();
    return 0;
}