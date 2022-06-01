#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <iostream>

class ImageConverterAR {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    image_transport::CameraSubscriber camera_sub_;
    tf::TransformBroadcaster br_;

public:
    ImageConverterAR(): it_(nh_) {
        image_pub_ = it_.advertise("/camera/markers", 10);
        camera_sub_ = it_.subscribeCamera("/camera/color/image_raw", 10, boost::bind(&ImageConverterAR::imageCb, this, _1, _2));
    }

    void imageCb(const sensor_msgs::ImageConstPtr &inImage,
                 const sensor_msgs::CameraInfoConstPtr &camera_info) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(inImage, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImage cv_image_out;
        cv_image_out.header = inImage->header;
        cv_image_out.encoding = "rgb8";
        cv_image_out.image = cv::Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
        cv::aruco::detectMarkers(cv_ptr->image, dictionary, markerCorners, markerIds);

        if (markerIds.size() > 0)
            cv::aruco::drawDetectedMarkers(cv_image_out.image, markerCorners, markerIds);

        cv::Mat cameraMatrix(3, 3, CV_64FC1, (void *) camera_info->K.data());
        cv::Mat distCoeffs(1, 5, CV_64FC1, (void *) camera_info->D.data());

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

        if (!rvecs.empty() && !tvecs.empty()) {
            cv::Vec3d rvec = rvecs[0];
            cv::Vec3d tvec = tvecs[0];
    
            inversePerspective(rvec, tvec);
    
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(tvec[0], tvec[1], tvec[2]));
            tf::Quaternion q;
            q.setRPY(rvec[0], rvec[1], rvec[2]);
            transform.setRotation(q);
            br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "marker_frame", "camera_frame"));
        }
        
        sensor_msgs::Image outMsg;
        cv_image_out.toImageMsg(outMsg);

        image_pub_.publish(outMsg);
    }

    void inversePerspective(cv::Vec3d& rvec, cv::Vec3d& tvec) {
        cv::Mat R(3, 3, CV_64FC1);
        cv::Rodrigues(rvec, R);
        R = R.t();
        cv::Rodrigues(R, rvec);
        tvec[0] = -tvec[0];
        tvec[1] = -tvec[1];
        tvec[2] = -tvec[2];
    }
};