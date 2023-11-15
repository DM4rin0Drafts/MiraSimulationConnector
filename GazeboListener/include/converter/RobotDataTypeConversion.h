
#pragma once

#include <image/Img.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace cv { namespace converter {

template <typename T>
cv::Mat convertImage(const T& img, const std::string& encoding) {
    // Convert the ROS image to a cv::Mat
    cv_bridge::CvImagePtr cvPtr;
    try {
        cvPtr = cv_bridge::toCvCopy(img, encoding);

        return cvPtr->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

}}
