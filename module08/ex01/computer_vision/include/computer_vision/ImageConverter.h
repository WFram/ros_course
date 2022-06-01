#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <computer_vision/ImgProcessingConfig.h>

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
    image_transport::Publisher image_c_b_pub_;
    image_transport::Publisher image_gamma_pub_;
    image_transport::Subscriber image_sub_;

    int alpha = 5, beta = 0;
    double gamma = 2.0, invGamma = 0.45;

public:
    ImageConverterCorrection(): it_(nh_) {
        image_sub_ = it_.subscribe("/image/origin", 1, boost::bind(&ImageConverterCorrection::imageCb, this, _1));
        image_c_b_pub_ = it_.advertise("/image/contrast_brightness", 1);
        image_gamma_pub_ = it_.advertise("/image/gamma", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &inMsg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(inMsg, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImage cv_image_c_b;
        cv_image_c_b.header = inMsg->header;
        cv_image_c_b.encoding = "rgb8";
        cv_image_c_b.image = cv::Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());
        for (int y = 0; y < cv_ptr->image.rows; y++) {
            for (int x = 0; x < cv_ptr->image.cols; x++) {
                for (int c = 0; c < cv_ptr->image.channels(); c++) {
                    cv_image_c_b.image.at<cv::Vec3b>(y, x)[c] =
                        cv::saturate_cast<uchar>(alpha * cv_ptr->image.at<cv::Vec3b>(y, x)[c] + beta);
                }
            }
        }

        cv_bridge::CvImage cv_image_gamma;
        cv_image_gamma.header = inMsg->header;
        cv_image_gamma.encoding = "rgb8";
        cv_image_gamma.image = cv::Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());
        
        cv::Mat table(1, 256, CV_8U);
        uchar *p = table.ptr();

        for (int i = 0; i < 256; ++i) {
            p[i] = cv::saturate_cast<uchar>(cv::pow(i / 255.0, invGamma) * 255.0);
        }

        cv::LUT(cv_ptr->image, table, cv_image_gamma.image);

        // for (int y = 0; y < cv_ptr->image.rows; y++) {
        //     for (int x = 0; x < cv_ptr->image.cols; x++) {
        //         for (int c = 0; c < cv_ptr->image.channels(); c++) {
        //             cv_image_gamma.image.at<cv::Vec3b>(y, x)[c] =
        //                 (uchar) (std::pow(cv_ptr->image.at<cv::Vec3b>(y, x)[c] / 255.0, invGamma) * 255);
        //         }
        //     }
        // }

        sensor_msgs::Image outMsgCB, outMsgGamma;
        cv_image_c_b.toImageMsg(outMsgCB);
        cv_image_gamma.toImageMsg(outMsgGamma);

        image_c_b_pub_.publish(outMsgCB);
        image_gamma_pub_.publish(outMsgGamma);
    }

    void serverCb(computer_vision::ImgProcessingConfig &config, uint32_t level) {
        alpha = config.contrast;
        beta = config.brightness;
        gamma = config.gamma_correction;
        invGamma = 1.0 / gamma;
    }
};