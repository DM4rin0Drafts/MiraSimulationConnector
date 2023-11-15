

#include <converter/MIRAAdapter.h>
#include <converter/RobotDataTypeConversion.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace mira { namespace ros {

MIRAAdapter::MIRAAdapter(int argc, char **argv, const std::string& namespaceName, const std::string& authorityName) 
{
    mFramework = new Framework(argc, argv, true);
    mAuthority = new Authority(namespaceName, authorityName);
}

MIRAAdapter::~MIRAAdapter()
{
    delete mFramework;
    delete mAuthority;
}

void MIRAAdapter::convertColorImage(const std::string& iChannelNameMIRA, const sensor_msgs::Image& iCameraImage) {
    mCameraImageChannel = mAuthority->publish<Img<void, 1>>(iChannelNameMIRA);

    cv::Mat image = cv::converter::convertImage<sensor_msgs::Image>(iCameraImage, sensor_msgs::image_encodings::BGR8);
    Img<void, 1> cameraImage(image);

    // publish camera image to mira
    publish(mCameraImageChannel, cameraImage, Time::now(), "", iCameraImage.step);
}

void MIRAAdapter::convertCompressedDepth(const std::string& iChannelNameMIRA, const sensor_msgs::CompressedImage& iCompressedDepth) 
{
    std::cout << "hier" << std::endl;
    mDepthImageChannel = mAuthority->publish<Img<void, 1>>(iChannelNameMIRA);
    // cv::Mat image = cv::converter::convertImage<sensor_msgs::CompressedImage>(iCompressedDepth, sensor_msgs::image_encodings::TYPE_32FC1);
    // Img<void, 1> cameraImage(image);

    // publish camera image to mira
    // publish(mDepthImageChannel, cameraImage, Time::now(), "", 0);

}

void MIRAAdapter::convertTheora(const std::string& iChannelNameMIRA, const theora_image_transport::Packet& iTheora) {

}


void MIRAAdapter::convertImu(const std::string& iChannelNameMIRA, const sensor_msgs::Imu& iImu) {

}


void MIRAAdapter::convertJointStates(const std::string& iChannelNameMIRA, const sensor_msgs::JointState& iJointState) {

}

void MIRAAdapter::convertOdometry(const std::string& iChannelNameMIRA, const nav_msgs::Odometry& iOdometry) 
{

}

template <typename T>
void MIRAAdapter::publish(Channel<T>& publisher, T& data, 
                            const Time& timestamp, const std::string& frameID, 
                            const unsigned int& sequenceID) {
    auto write = publisher.write();
    write->timestamp = timestamp;
    write->frameID = frameID;
    write->sequenceID = sequenceID;
    write->value() = data;

}



}}