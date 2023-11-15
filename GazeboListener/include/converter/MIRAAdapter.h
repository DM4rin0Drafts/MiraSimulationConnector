


#pragma once

#include <iostream>
#include <map>

#include <fw/Framework.h>
#include <image/Img.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <theora_image_transport/Packet.h>

namespace mira { namespace ros {

typedef std::vector<std::string> ChannelMap;

class MIRAAdapter {
public:
    MIRAAdapter(int argc, char **argv, 
                const std::string& namespaceName, const std::string& authorityName);
    ~MIRAAdapter();

    void convertColorImage(const std::string& iChannelNameMIRA, 
                           const sensor_msgs::Image& iColorImage);
    void convertCompressedDepth(const std::string& iChannelNameMIRA, 
                                const sensor_msgs::CompressedImage& iCompressedDepth); 
    void convertTheora(const std::string& iChannelNameMIRA, 
                       const theora_image_transport::Packet& iTheora);
    void convertImu(const std::string& iChannelNameMIRA, 
                    const sensor_msgs::Imu& iImu);             
    void convertJointStates(const std::string& iChannelNameMIRA, 
                            const sensor_msgs::JointState& iJointState);
    void convertOdometry(const std::string& iChannelNameMIRA, 
                         const nav_msgs::Odometry& iOdometry);

private:
    template <typename T>
    void publish(Channel<T>& publisher, T& data, 
                 const Time& timestamp, const std::string& frameID, 
                 const unsigned int& sequenceID);

private:
    Channel<Img<void, 1>> mCameraImageChannel;
    Channel<Img<void, 1>> mDepthImageChannel;


private:
    ChannelMap mROSChannelList;
    ChannelMap mMIRAChannelList;
    
    Authority* mAuthority;
    Framework* mFramework;
};

}}