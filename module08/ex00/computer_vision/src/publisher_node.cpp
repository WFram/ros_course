#include <computer_vision/ImageConverter.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "publisher_node");

    ros::NodeHandle nh;
    std::string img_file;

    nh.param<std::string>("img_file", img_file, "");

    cv::Mat img;
    img = cv::imread(img_file.c_str(), cv::IMREAD_COLOR);

    ImageConverterPub ic;

    ros::Rate r(10);

    while (ros::ok()) {
        cv_bridge::CvImage cvImage;
        cvImage.encoding = "rgb8";
        cvImage.image = img;

        sensor_msgs::Image imageMsg;
        cvImage.toImageMsg(imageMsg);

        ic.Publish(imageMsg);

        r.sleep();
    }

    return 0;
}