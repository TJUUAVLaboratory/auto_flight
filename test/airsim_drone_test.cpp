// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <ros/ros.h>
#include "ros_traj_plan/uavtrans.h"
#include "mavros/frame_tf.h"
#include <iostream>
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>

using namespace msr::airlib;

typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;
typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaterniond;
typedef Eigen::Vector3d Vector3d;
MultirotorRpcLibClient client("172.20.132.59");
Vector3d position;
Vector3d position_ned;
Vector3d waypoint_ned(0, 0, 0);
//Quaternionr quaternion;
Quaterniond quaternion_enu(1, 0, 0, 0);
Quaterniond quaternion_ned(1, 0, 0, 0);

float fly_x; // current position (ENU coordinate system).
float fly_y; // current position (ENU coordinate system).
float fly_z; // current position (ENU coordinate system).
double quat_w, quat_x, quat_y, quat_z;
Vector3d airsim_rpy;

float fly_roll, fly_pitch, fly_yaw;
float fly_roll_ned, fly_pitch_ned, fly_yaw_ned;

void image_capture()
{
    //std::cout << "Press Enter to get FPV image" << std::endl; std::cin.get();
    vector<ImageRequest> request = {ImageRequest(0, ImageType::Scene), ImageRequest(1, ImageType::Scene), ImageRequest(2, ImageType::Scene)};
    const vector<ImageResponse> &response = client.simGetImages(request);
    std::cout << "# of images recieved: " << response.size() << std::endl;

    if (response.size() > 0)
    {
        //std::cout << "Enter path with ending separator to save images (leave empty for no save)" << std::endl;
        std::string path("/home/wuqi/airpic");
        //std::getline(std::cin, path);

        for (const ImageResponse &image_info : response)
        {
            std::cout << "Image uint8 size: " << image_info.width << "*" << image_info.height << std::endl;
            //std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;

            if (path != "")
            {
                std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
                if (image_info.pixels_as_float)
                {
                    Utils::writePfmFile(image_info.image_data_float.data(), image_info.width, image_info.height,
                                        file_path + ".pfm");
                }
                else
                {
                    std::ofstream file(file_path + ".png", std::ios::binary);
                    file.write(reinterpret_cast<const char *>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
                    file.close();
                }
            }
        }
    }
}

void display_transform_position()
{
    position_ned = Vector3d(client.getPosition().x(), client.getPosition().y(), client.getPosition().z());
    position = mavros::ftf::transform_frame_ned_enu(Vector3d(client.getPosition().x(), client.getPosition().y(), client.getPosition().z()));
    quaternion_ned = Quaterniond(client.getOrientation().w(), client.getOrientation().x(), client.getOrientation().y(), client.getOrientation().z());
    quaternion_enu = mavros::ftf::transform_orientation_ned_enu(mavros::ftf::transform_orientation_aircraft_baselink(quaternion_ned));
    fly_x = position.x(); // current position (ENU coordinate system).
    fly_y = position.y(); // current position (ENU coordinate system).
    fly_z = position.z(); // current position (ENU coordinate system).
    waypoint_ned = mavros::ftf::transform_frame_enu_ned(position);
    airsim_rpy = mavros::ftf::quaternion_to_rpy(quaternion_enu);
    fly_roll = airsim_rpy(0);
    fly_pitch = airsim_rpy(1);
    fly_yaw = airsim_rpy(2);
    VectorMathf::toEulerianAngle(client.getOrientation(), fly_roll_ned, fly_pitch_ned, fly_yaw_ned);

    std::cout << "flight positio(ENU):\n:" << fly_x << "\ny:" << fly_y << "\nz:" << fly_z << std::endl;
    std::cout << "flight positio(NED):\n:" << position_ned << std::endl;
    std::cout << "flight oritation(ENU):\nroll:" << fly_roll * 180 / 3.1415 << "\npitch:" << fly_pitch * 180 / 3.1415 << "\nyaw:" << fly_yaw * 180 / 3.1415 << std::endl;
    std::cout << "\nflight oritation(NED):\nroll:" << fly_roll_ned * 180 / 3.1415 << "\npitch:" << fly_pitch_ned * 180 / 3.1415 << "\nyaw:" << fly_yaw_ned * 180 / 3.1415 << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "drone_test");
    ros::NodeHandle hd_nh;

    client.confirmConnection();

    std::cout << "Press Enter to arm the drone" << std::endl;
    std::cin.get();
    client.enableApiControl(true);
    client.armDisarm(true);

    std::cout << "Press Enter to takeoff" << std::endl;
    std::cin.get();
    float takeoffTimeout = 5;
    client.takeoff(takeoffTimeout);

    // switch to explicit hover mode so that this is the fallback when
    // move* commands are finished.
    //std::this_thread::sleep_for(std::chrono::duration<double>(5));
    client.hover();
    //image_capture();

    //std::cout << "Press Enter to fly in a 10m box pattern at 3 m/s velocity" << std::endl; std::cin.get();
    // moveByVelocityZ is an offboard operation, so we need to set offboard mode.
    client.enableApiControl(true);
    //auto position = client.getPosition();

    // quaternion = client.getOrientation();
    // quat_w = quaternion.w();
    // quat_x = quaternion.x();
    // quat_y = quaternion.y();
    // quat_z = quaternion.z();
    // std::cout << "四元数：" << quat_w << "  " << quat_x << "  " << quat_y << "  " << quat_z << "  " << std::endl;

    display_transform_position();

    const float speed = 3.0f;
    const float size = 10.0f;
    const float duration = size / speed;
    float max_wait_seconds = 10.0f;
    DrivetrainType driveTrain = DrivetrainType::ForwardOnly;
    YawMode yaw_mode(true, 0);
    YawMode yaw_mode_position(false, 0);
    client.moveToPosition(waypoint_ned.x(), waypoint_ned.y(), waypoint_ned.z() - 6.0f, 3.0f, max_wait_seconds, driveTrain, yaw_mode_position, -1.0f, 1.0f);

    image_capture();

    display_transform_position();

    for (int p_i = 0; p_i <= 10000; p_i++)
    {
        client.moveToPosition(waypoint_ned.x() + 25, waypoint_ned.y() - 15, waypoint_ned.z() - 6, 3.0f, 0.1f, driveTrain, yaw_mode_position, -1.0f, 1.0f);
        display_transform_position();
        std::cout << "time=" << p_i * 10 << "ms" << std::endl;
        delay_msec(10.0f);
        image_capture();
    }
    image_capture();

    display_transform_position();

    client.hover();

    std::cout << "Press Enter to go home" << std::endl;
    std::cin.get();
    client.reset();
    client.goHome();

    std::cout << "Press Enter to land" << std::endl;
    std::cin.get();
    client.land(20);

    std::cout << "Press Enter to disarm" << std::endl;
    std::cin.get();
    client.armDisarm(false);

    return 0;
}
