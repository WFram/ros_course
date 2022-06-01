#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <iostream>

class ImageConverterPub {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

public:
    ImageConverterPub(): it_(nh_) {
        image_pub_ = it_.advertise("/image/color", 1);
    }

    void Publish(sensor_msgs::Image &msg) { image_pub_.publish(msg); };
};

class ImageConverterSwitch {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    image_transport::Subscriber image_sub_;

public:
    ImageConverterSwitch(): it_(nh_) {
        image_sub_ = it_.subscribe("/image/color", 1, boost::bind(&ImageConverterSwitch::imageCb, this, _1));
        image_pub_ = it_.advertise("/image/gray", 1);
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

        cv_bridge::CvImage cv_image;
        cv_image.header = inMsg->header;
        cv_image.encoding = "mono8";
        cv_image.image = cv_ptr->image;

        sensor_msgs::Image outMsg;
        cv_image.toImageMsg(outMsg);

        image_pub_.publish(outMsg);
    }
};