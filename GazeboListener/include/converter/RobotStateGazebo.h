

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <theora_image_transport/Packet.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>

#include <converter/MIRAAdapter.h>

namespace ros {

class RobotStateGazebo : public mira::ros::MIRAAdapter {
public:
    RobotStateGazebo(int argc, char **argv, 
                const std::string& namespaceName, const std::string& authorityName); //(int argc, char **argv, const std::string& namespaceName, const std::string& authorityName);
    ~RobotStateGazebo();

private:
    void onColorImage(const sensor_msgs::Image& iColorImage);
    void onCompressedDepth(const sensor_msgs::CompressedImage& iCompressedDepth);
    void onTheora(const theora_image_transport::Packet& iTheora);
    void onImu(const sensor_msgs::Imu& iImu);
    void onJointStates(const sensor_msgs::JointState& iJointState);
    void onOdometry(const nav_msgs::Odometry& iOdometry);
    void onScan(sensor_msgs::LaserScan iLaserScan);
    void onTf(tf2_msgs::TFMessage iTf);

private:
    sensor_msgs::Image mSimulationColorImage;

    NodeHandle* mNodeHandler;
    
    Subscriber mROSColorImage;
    Subscriber mROSCompressedDepth;
    Subscriber mROSTheora;
    Subscriber mROSImu;
    Subscriber mROSJointStates;
    Subscriber mROSOdometry;
    Subscriber mROSLaserScan;
    Subscriber mROSTf;
};

}