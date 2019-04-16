#include <ros/ros.h>
#include "mavros/frame_tf.h"
#include <iostream>
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>

using namespace std;
typedef Eigen::Quaternion<double> Quaterniond;
typedef Eigen::Vector3d Vector3d;

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "enu_neu_frame2");
    ros::NodeHandle frame_nh;

    msr::airlib::MultirotorRpcLibClient client("192.168.0.210");

    Vector3d position;
    Quaterniond quaternion_enu(1, 0, 0, 0);
    Quaterniond quaternion_ned(1, 0, 0, 0);
    Vector3d airsim_rpy;
    float fly_roll, fly_pitch, fly_yaw;
    quaternion_enu = mavros::ftf::transform_orientation_ned_enu(mavros::ftf::transform_orientation_baselink_aircraft(quaternion_ned));
    airsim_rpy = mavros::ftf::quaternion_to_rpy(quaternion_enu);
    fly_roll = airsim_rpy(0);
    fly_pitch = airsim_rpy(1);
    fly_yaw = airsim_rpy(2);
    std::cout << "flight oritation:\nroll:" << fly_roll * 180 / 3.1415 << "\npitch:" << fly_pitch * 180 / 3.1415 << "\nyaw:" << fly_yaw * 180 / 3.1415 << std::endl;
    client.confirmConnection();
    return 0;
}