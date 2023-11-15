
#include <iostream>

// ros packages include
#include <ros/ros.h>

// mira packages include
#include <fw/Framework.h>

//#include <converter/GazeboAdapter.h>
#include <converter/RobotStateGazebo.h>

int main(int argc, char **argv) {
    // mira::Authority authority("/simulation/", "gazebo_listener");

    int mRosArgc = 1;
	char **mRosArgv;
    
    mRosArgv = new char *[mRosArgc];
	mRosArgv[0] = "localhost:11311";
    
    ros::init(argc, argv, "gazebo_listener");

    std::vector<std::string> mROSChannels = {"/camera/image"};
    std::vector<std::string> mMIRAChannels = {"/hardware/ColorImage"};

    ros::RobotStateGazebo test(argc, argv, "/simulation/", "GazeboListener");


    ros::spin();
    return 0;
}