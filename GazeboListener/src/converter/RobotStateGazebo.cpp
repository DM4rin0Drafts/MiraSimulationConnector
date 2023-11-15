
#include <converter/RobotStateGazebo.h>

namespace ros {

RobotStateGazebo::RobotStateGazebo(int argc, char **argv, 
                const std::string& namespaceName, const std::string& authorityName) : mira::ros::MIRAAdapter(argc, argv, namespaceName, authorityName) {
    mNodeHandler = new NodeHandle;

    mROSColorImage = mNodeHandler->subscribe("/camera/image", 20, &RobotStateGazebo::onColorImage, this);
    // mROSCompressedDepth = mNodeHandler->subscribe("/camera/image/compressedDepth", 20, &RobotStateGazebo::onCompressedDepth, this);
    mROSTheora = mNodeHandler->subscribe("/camera/image/theora", 20, &RobotStateGazebo::onTheora, this);
    mROSImu = mNodeHandler->subscribe("/imu", 20, &RobotStateGazebo::onImu, this);
    mROSJointStates = mNodeHandler->subscribe("/joint_states", 20, &RobotStateGazebo::onJointStates, this);
    mROSOdometry = mNodeHandler->subscribe("/odom", 20, &RobotStateGazebo::onOdometry, this);
    mROSLaserScan = mNodeHandler->subscribe("/scan", 20, &RobotStateGazebo::onScan, this);
    mROSTf = mNodeHandler->subscribe("/tf", 20, &RobotStateGazebo::onTf, this);
}

RobotStateGazebo::~RobotStateGazebo() {}

void RobotStateGazebo::onColorImage(const sensor_msgs::Image& iColorImage) 
{
    convertColorImage("/hardware/ColorImage", iColorImage);
}

void RobotStateGazebo::onCompressedDepth(const sensor_msgs::CompressedImage& iCompressedDepth)
{
    convertCompressedDepth("/hardware/DepthImage", iCompressedDepth);
}

void RobotStateGazebo::onTheora(const theora_image_transport::Packet& iTheora) 
{
    /*
    ---
header: 
  seq: 1852
  stamp: 
    secs: 1944
    nsecs:  97000000
  frame_id: "camera_rgb_optical_frame"
data: [95, 169, 161, 248, 3, 44, 95, 192, 145, 248, 40, 60, 28, 159, 68, 60, 31, 67, 147, 196, 170, 253, 24, 56, 145, 88, 251, 193, 120, 185, 56, 25, 23, 208, 64]
b_o_s: 0
e_o_s: 0
granulepos: 114809
packetno: 1852

    */
    //std::cout << "theora" << std::endl;
    convertTheora("/hardware/Theora", iTheora);
}

void RobotStateGazebo::onImu(const sensor_msgs::Imu& iImu)
{
    /*
    eader: 
  seq: 67837
  stamp: 
    secs: 1205
    nsecs: 816000000
  frame_id: "base_footprint"
orientation: 
  x: -0.003499308826800902
  y: 0.0016486873015084634
  z: 0.9034993859598007
  w: 0.4285719265633011
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity: 
  x: -0.0007196996369392019
  y: 0.0007912901823340856
  z: 0.13051714286624733
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration: 
  x: 5.932967252840097e-05
  y: 0.001135564627608273
  z: -6.473454671423164e-06
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    */
    //std::cout << "imu" << std::endl;
    convertImu("/hardware/Imu", iImu);
}

void RobotStateGazebo::onJointStates(const sensor_msgs::JointState& iJointState)
{

    /*
    header: 
  seq: 31387
  stamp: 
    secs: 1162
    nsecs: 762000000
  frame_id: ''
name: 
  - wheel_right_joint
  - wheel_left_joint
position: [1.5633708856927218, -1.5777286355376283]
velocity: []
effort: []

    */
    //std::cout << "jointstates" << std::endl;
    convertJointStates("/robot/JointState", iJointState);
}

void RobotStateGazebo::onOdometry(const nav_msgs::Odometry& iOdometry)
{
    //std::cout << "odometry" << std::endl;
    /*
    header: 
  seq: 33873
  stamp: 
    secs: 1245
    nsecs: 629000000
  frame_id: "odom"
child_frame_id: "base_footprint"
pose: 
  pose: 
    position: 
      x: 0.2052929940607495
      y: -1.7636745664071218
      z: -0.0010034380362699026
    orientation: 
      x: 0.002151565017314862
      y: -0.0032146526994626013
      z: -0.5540328604330951
      w: -0.8324858114924967
  covariance: [1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
twist: 
  twist: 
    linear: 
      x: -0.0013053712159538243
      y: 0.008627086688404426
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.13072270902930666
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    */
    convertOdometry("/robot/Odometry", iOdometry);
}


void RobotStateGazebo::onScan(sensor_msgs::LaserScan iLaserScan) {
/*    header: 
  seq: 8
  stamp: 
    secs: 1301
    nsecs: 123000000
  frame_id: "base_scan"
angle_min: 0.0
angle_max: 6.28318977355957
angle_increment: 0.017501922324299812
time_increment: 0.0
scan_time: 0.0
range_min: 0.11999999731779099
range_max: 3.5
ranges: [0.2819483280181885, 0.29539579153060913, 0.28642696142196655, 0.28522294759750366, 0.26588529348373413, 0.2628369927406311, 0.2370147407054901, 0.24690847098827362, 0.25262686610221863, 0.24206630885601044, 0.24677015841007233, 0.2556396424770355, 0.23805660009384155, 0.2717360258102417, 0.250404417514801, 0.2666090130805969, 1.6676702499389648, 1.673054575920105, 1.723020076751709, 2.6446664333343506, 2.6236274242401123, 0.8526315093040466, 0.8427931666374207, 0.822060227394104, 0.8191267251968384, 0.821418821811676, 0.8279629349708557, 0.8047571182250977, 0.807858943939209, 0.8229496479034424, 0.8409528136253357, 0.8323255777359009, 0.8726154565811157, 2.2929322719573975, 2.2678632736206055, 2.2523629665374756, 2.222073793411255, 2.212632656097412, 2.20198130607605, 2.1954686641693115, 1.5144189596176147, 1.500195860862732, 1.4810662269592285, 1.483040690422058, 1.4809781312942505, 1.4824352264404297, 1.5087960958480835, 2.119642972946167, 2.114553928375244, 2.1024839878082275, 2.0899531841278076, 2.082202911376953, 2.097618579864502, 2.0828680992126465, 2.0879571437835693, 2.0909223556518555, 2.0940871238708496, 2.0810000896453857, 2.0841596126556396, 2.0844273567199707, 2.0718114376068115, 2.0771877765655518, 1.6678053140640259, 1.3984248638153076, 1.1557191610336304, 1.035009741783142, 0.9100199341773987, 0.8334190249443054, 0.7505364418029785, 0.702436625957489, 0.6349225640296936, 0.5822256207466125, 0.5503003597259521, 0.5203922390937805, 0.4825513958930969, 0.45012450218200684, 0.4389515519142151, 0.4232527017593384, 0.4061237573623657, 0.38363516330718994, 0.3795759081840515, 0.33487802743911743, 0.342191606760025, 0.3326282203197479, 0.3077327609062195, 0.29801222681999207, 0.31386598944664, 0.2784789204597473, 0.26272153854370117, 0.2805265486240387, 0.27748140692710876, 0.2540128529071808, 0.2197587490081787, 0.23398466408252716, 0.2353678047657013, 0.2225772887468338, 0.23804530501365662, 0.23637999594211578, 0.20306503772735596, 0.21955566108226776, 0.2151980698108673, 0.20958836376667023, 0.19603319466114044, 0.1899595558643341, 0.20088335871696472, 0.1823563277721405, 0.2062854766845703, 0.19479216635227203, 0.20182380080223083, 0.2175009548664093, 0.2171526700258255, 0.2110690474510193, 0.222204327583313, 0.22032281756401062, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 3.4924426078796387, 3.499274253845215, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 1.6297367811203003, 1.3743807077407837, 1.19280207157135, 1.0486677885055542, 0.941818118095398, 0.8507789969444275, 0.7736769914627075, 0.7232872843742371, 0.6690799593925476, 0.623885989189148, 0.5738626718521118, 0.5540060997009277, 0.5181089639663696, 0.5037035942077637, 0.45915448665618896, 0.4378664195537567, 0.43381214141845703, 0.4227778911590576, 0.38465505838394165, 0.3690236806869507, 0.3614150583744049, 0.34288957715034485, 0.3131949007511139, 0.30958274006843567, 0.31133660674095154, 0.30375298857688904, 0.30266815423965454, 0.2955723702907562]
intensities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
*/

}

void RobotStateGazebo::onTf(tf2_msgs::TFMessage iTf) {
    /*
    ^Ctransforms: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1520
        nsecs: 495000000
      frame_id: "odom"
    child_frame_id: "base_footprint"
    transform: 
      translation: 
        x: 0.24874052419857742
        y: -1.8582371890675176
        z: -0.0010034403117488572
      rotation: 
        x: 0.0011584302106248474
        y: 0.003690725096807632
        z: -0.30195598853362077
        w: 0.9533140183467307
---

    */
}

}