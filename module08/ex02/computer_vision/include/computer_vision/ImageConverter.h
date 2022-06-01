#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <computer_vision/CannyDetectionConfig.h>

#include <string>
#include <iostream>

class ImageConverterPub {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

public:
    ImageConverterPub(): it_(nh_) {
        image_pub_ = it_.advertise("/image/origin", 1);
    }

    void Publish(sensor_msgs::Image &msg) { image_pub_.publish(msg); };
};

class ImageConverterCorrection {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_edge_pub_;
    image_transport::Subscriber image_sub_;

    int kernel_size_blur = 5, kernel_size_canny = 5, th_low = 100, th_high = 400;

public:
    ImageConverterCorrection(): it_(nh_) {
        image_sub_ = it_.subscribe("/image/origin", 1, boost::bind(&ImageConverterCorrection::imageCb, this, _1));
        image_edge_pub_ = it_.advertise("/image/canny", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &inMsg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(inMsg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImage cv_image_edge;
        cv_image_edge.header = inMsg->header;
        cv_image_edge.encoding = "mono8";
        cv_image_edge.image = cv::Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());

        cv::GaussianBlur(cv_ptr->image, cv_image_edge.image, cv::Size(kernel_size_blur, kernel_size_blur), 0, 0);

        cv::Canny(cv_image_edge.image, cv_image_edge.image, th_low, th_high, kernel_size_canny);

        sensor_msgs::Image outMsg;
        cv_image_edge.toImageMsg(outMsg);

        image_edge_pub_.publish(outMsg);
    }

    void serverCb(computer_vision::CannyDetectionConfig &config, uint32_t level) {
        kernel_size_blur = config.size_blur_kernel;
        kernel_size_canny = config.size_canny_kernel;
        th_low = config.low;
        th_high = config.high;
    }
};