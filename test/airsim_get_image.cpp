// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <ros/ros.h>
#include "ros_traj_plan/uavtrans.h"
#include "mavros/frame_tf.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "ros_traj_plan/airsimcommon.h"

using namespace msr::airlib;

class airsim_image
{
  public:
    airsim_image(ros::NodeHandle nh) : nh_img(nh),
                                       it(nh_img),
                                       client("")
    {
        //MultirotorRpcLibClient client =  MultirotorRpcLibClient("172.20.205.46");
        pub_image_left = it.advertise("/zed/left/image_rect_color", 1);
        frame_left = boost::make_shared<cv_bridge::CvImage>();
        frame_left->encoding = sensor_msgs::image_encodings::RGB8;
        //frame_left->encoding = sensor_msgs::image_encodings::BGR8;

        pub_image_right = it.advertise("/zed/right/image_rect_color", 1);
        frame_right = boost::make_shared<cv_bridge::CvImage>();
        frame_right->encoding = sensor_msgs::image_encodings::RGB8;
        //frame_right->encoding = sensor_msgs::image_encodings::BGR8;

        pub_image_DisparityNormalized = it.advertise("/zed/DisparityNormalized/image_rect_color", 1);
        frame_DisparityNormalized = boost::make_shared<cv_bridge::CvImage>();
        frame_DisparityNormalized->encoding = sensor_msgs::image_encodings::TYPE_32FC1; //RGB8; //MONO16

        //airsim_img_request = vector<ImageRequest>{ImageRequest(1, ImageType::Scene, false, true), ImageRequest(2, ImageType::Scene, false, true)};
        airsim_img_request = vector<ImageRequest>{ImageRequest(2, ImageType::Scene, false, false), ImageRequest(1, ImageType::Scene, false, false),
                                                  ImageRequest(2, ImageType::DisparityNormalized, true)};
        airsim_depthimg_request = vector<ImageRequest>{ImageRequest(2, ImageType::DisparityNormalized, true,false)};

        image_odometry_publisher = nh_img.advertise<nav_msgs::Odometry>("/zed/odom", 1);
        Disparity_publisher = nh_img.advertise<std_msgs::Float32MultiArray>("/zed/Disparity", 1);
    }

    void client_connect()
    {
        client.confirmConnection();
    }

    void airsim_image_capture()
    {
        airsim_img_response = client.simGetImages(airsim_img_request);
        //airsim_depthimg_response = client.simGetImages(airsim_depthimg_request);

        //rgba_img = cv::Mat(airsim_img_response.at(0).height, airsim_img_response.at(0).width, CV_8UC4);
        //std::string path("/home/ubuntu/wuqi/airpic/");
        //ROS_INFO_STREAM( "# of images recieved: " << airsim_img_response.size());
        //std::cout << "# of images recieved: " << airsim_img_response.size() << std::endl;

        for (int m = 0; m < airsim_img_response.size(); m++)
        {
            rgba_img.release();
            new_rgb_img.release();
            //rgb_img.convertTo(new_rgb_img, -1, 1.5, -10);

            //new_rgb_img= cv::imdecode(airsim_img_response.at(m).image_data_uint8, 1);

            // rgba_img = cv::Mat(image_info.height, image_info.width, CV_32FC1);
            //memcpy(rgba_img.data, image_info.image_data_float.data(),image_info.image_data_float.size()*sizeof(float));

            //ROS_INFO_STREAM("image row*cols:"<<rgba_img.rows<<"*"<<rgba_img.cols);
            if (m == 0)
            {
                rgba_img = cv::Mat(airsim_img_response.at(m).height, airsim_img_response.at(m).width, CV_8UC4);
                memcpy(rgba_img.data, reinterpret_cast<const char *>(airsim_img_response.at(m).image_data_uint8.data()), airsim_img_response.at(m).image_data_uint8.size() * sizeof(char));
                cv::cvtColor(rgba_img, new_rgb_img, CV_RGBA2RGB);
                //file_path = path + "left" + std::to_string(airsim_img_response.at(m).time_stamp);
                //frame_left->image = rgba_img;
                frame_left->image = new_rgb_img;
                frame_left->header.stamp = ros::Time::now();
                pub_image_left.publish(frame_left->toImageMsg());
                //cv::imshow("left image", new_rgb_img);
                //std::cout<<"left:"<<m<<std::endl;
            }
            else if (m == 1)
            {
                rgba_img = cv::Mat(airsim_img_response.at(m).height, airsim_img_response.at(m).width, CV_8UC4);
                memcpy(rgba_img.data, reinterpret_cast<const char *>(airsim_img_response.at(m).image_data_uint8.data()), airsim_img_response.at(m).image_data_uint8.size() * sizeof(char));
                cv::cvtColor(rgba_img, new_rgb_img, CV_RGBA2RGB);
                //file_path = path + "right" + std::to_string(airsim_img_response.at(m).time_stamp);
                //frame_right->image = rgba_img;
                frame_right->image = new_rgb_img;
                frame_right->header.stamp = ros::Time::now();
                pub_image_right.publish(frame_right->toImageMsg());
                //cv::imshow("rightimage",rgb_img);
                //std::cout<<"right:"<<m<<std::endl;
            }
            else if (m == 2)
            {
                rgba_img.release();
                new_rgb_img.release();
                gray_img = cv::Mat(airsim_img_response.at(m).height, airsim_img_response.at(m).width, CV_32FC1);
                memcpy(gray_img.data, reinterpret_cast<const float *>(airsim_img_response.at(m).image_data_float.data()), airsim_img_response.at(m).image_data_float.size() * sizeof(float));
                //cv::cvtColor(rgba_img, new_rgb_img, CV_RGBA2GRAY);
                //new_rgb_img= cv::imdecode(airsim_img_response.at(m).image_data_float, 0);
                frame_DisparityNormalized->image = gray_img;
                frame_DisparityNormalized->header.stamp = ros::Time::now();
                pub_image_DisparityNormalized.publish(frame_DisparityNormalized->toImageMsg());
                // for(int i = 0;i <airsim_img_response.at(m).image_data_float.size();i++)
                // {
                //     Disparity_array.data.push_back(airsim_img_response.at(m).image_data_float[i]);
                // }
                Disparity_array.data.assign(airsim_img_response.at(m).image_data_float.begin(), airsim_img_response.at(m).image_data_float.end()); // = airsim_img_response.at(m).image_data_float.data();
                Disparity_publisher.publish(Disparity_array);
                //std::cout << "Disparity float value:" << Disparity_array.data.at(40000) << std::endl;
                //cv::imshow("DepthVis image",gray_img);
                //cv::waitKey(1);
               // std::cout << "depth float value:" << airsim_img_response.at(m).image_data_float.at(40000) << std::endl;
                //std::cout << "Image float size: " << airsim_img_response.at(m).image_data_float.size() << std::endl;
            }

            //cv::imwrite(file_path+".jpg",rgba_img);
            //cv::waitKey(1);

            //std::cout << "Image unit8 size: " << airsim_img_response.at(m).image_data_uint8.size() << std::endl;
            //std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;
        }

        // for (int m = 0; m < airsim_depthimg_response.size(); m++)
        // {
        //     rgba_img.release();
        //     new_rgb_img.release();
        //     rgba_img = cv::Mat(airsim_depthimg_response.at(m).height, airsim_depthimg_response.at(m).width, CV_8UC4);
        //     memcpy(rgba_img.data, reinterpret_cast<const char *>(airsim_depthimg_response.at(m).image_data_uint8.data()), airsim_depthimg_response.at(m).image_data_uint8.size() * sizeof(char));
        //     cv::cvtColor(rgba_img, new_rgb_img, CV_RGBA2RGB);
        //     frame_DisparityNormalized->image = new_rgb_img;
        //     frame_DisparityNormalized->header.stamp = ros::Time::now();
        //     pub_image_DisparityNormalized.publish(frame_DisparityNormalized->toImageMsg());
        //     std::cout << "depth float value:" << airsim_depthimg_response.at(m).image_data_uint8[50] << std::endl;
        //     std::cout << "Image float size: " << airsim_depthimg_response.at(m).image_data_uint8.size() << std::endl;
        // }

        position_ned = Vector3d(airsim_img_response.at(0).camera_position.x(), airsim_img_response.at(0).camera_position.y(),
                                airsim_img_response.at(0).camera_position.z());
        position_enu = mavros::ftf::transform_frame_ned_enu(position_ned);

        quaternion_ned = Quaterniond(airsim_img_response.at(0).camera_orientation.w(), airsim_img_response.at(0).camera_orientation.x(),
                                     airsim_img_response.at(0).camera_orientation.y(), airsim_img_response.at(0).camera_orientation.z());
        quaternion_enu = mavros::ftf::transform_orientation_ned_enu(mavros::ftf::transform_orientation_aircraft_baselink(quaternion_ned));

        image_odometry.pose.pose.position.x = position_enu.x();
        image_odometry.pose.pose.position.y = position_enu.y();
        image_odometry.pose.pose.position.z = position_enu.z() - 0.3;
        image_odometry.pose.pose.orientation.w = quaternion_enu.w();
        image_odometry.pose.pose.orientation.x = quaternion_enu.x();
        image_odometry.pose.pose.orientation.y = quaternion_enu.y();
        image_odometry.pose.pose.orientation.z = quaternion_enu.z();
        image_odometry.header.frame_id = "map";

        // airsim_rpy = mavros::ftf::quaternion_to_rpy(quaternion_enu);
        // fly_roll = airsim_rpy(0);
        // fly_pitch = airsim_rpy(1);
        // fly_yaw =  mavros::ftf::quaternion_get_yaw(quaternion_enu);
        // std::cout << "flight oritation(ENU):\nroll:" << fly_roll * 180 / 3.1415 << "\npitch:" << fly_pitch * 180 / 3.1415 << "\nyaw:" << fly_yaw * 180 / 3.1415 << std::endl;

        image_odometry_publisher.publish(image_odometry);
    }

    void airsim_Odometry()
    {
        position_ned = Vector3d(airsim_img_response.at(0).camera_position.x(), airsim_img_response.at(0).camera_position.y(),
                                airsim_img_response.at(0).camera_position.z());
        position_enu = mavros::ftf::transform_frame_ned_enu(position_ned);

        quaternion_ned = Quaterniond(airsim_img_response.at(0).camera_orientation.w(), airsim_img_response.at(0).camera_orientation.x(),
                                     airsim_img_response.at(0).camera_orientation.y(), airsim_img_response.at(0).camera_orientation.z());
        quaternion_enu = mavros::ftf::transform_orientation_ned_enu(mavros::ftf::transform_orientation_aircraft_baselink(quaternion_ned));

        image_odometry.pose.pose.position.x = position_enu.x();
        image_odometry.pose.pose.position.y = position_enu.y();
        image_odometry.pose.pose.position.z = position_enu.z() + 0.15;
        image_odometry.pose.pose.orientation.w = quaternion_enu.w();
        image_odometry.pose.pose.orientation.x = quaternion_enu.x();
        image_odometry.pose.pose.orientation.y = quaternion_enu.y();
        image_odometry.pose.pose.orientation.z = quaternion_enu.z();
        image_odometry.header.frame_id = "map";

        // airsim_rpy = mavros::ftf::quaternion_to_rpy(quaternion_enu);
        // fly_roll = airsim_rpy(0);
        // fly_pitch = airsim_rpy(1);
        // fly_yaw =  mavros::ftf::quaternion_get_yaw(quaternion_enu);
        // std::cout << "flight oritation(ENU):\nroll:" << fly_roll * 180 / 3.1415 << "\npitch:" << fly_pitch * 180 / 3.1415 << "\nyaw:" << fly_yaw * 180 / 3.1415 << std::endl;

        image_odometry_publisher.publish(image_odometry);
    }

    void image_capture()
    {
        //std::cout << "Press Enter to get FPV image" << std::endl; std::cin.get();
        vector<ImageRequest> request = {ImageRequest(1, ImageType::Scene, false, true), ImageRequest(2, ImageType::Scene, false, true)};
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

  private:
    /* data */
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;
    typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaterniond;
    typedef Eigen::Vector3d Vector3d;

    ros::NodeHandle nh_img;
    ros::Publisher image_odometry_publisher, Disparity_publisher;
    image_transport::ImageTransport it;
    image_transport::Publisher pub_image_left, pub_image_right, pub_image_DisparityNormalized;
    cv_bridge::CvImagePtr frame_left, frame_right, frame_DisparityNormalized;
    cv::Mat rgba_img, rgb_img, new_rgb_img, gray_img;
    std_msgs::Float32MultiArray Disparity_array;

    std::string file_path;

    MultirotorRpcLibClient client;

    std::vector<ImageRequest> airsim_img_request;
    std::vector<ImageResponse> airsim_img_response;
    std::vector<ImageRequest> airsim_depthimg_request;
    std::vector<ImageResponse> airsim_depthimg_response;
    std::vector<cv::Mat> img_lr;

    Vector3d position_enu;
    Vector3d position_ned;
    Vector3d waypoint_ned;
    //Quaternionr quaternion;
    Quaterniond quaternion_enu;
    Quaterniond quaternion_ned;
    nav_msgs::Odometry image_odometry;

    float fly_x; // current position (ENU coordinate system).
    float fly_y; // current position (ENU coordinate system).
    float fly_z; // current position (ENU coordinate system).
    double quat_w, quat_x, quat_y, quat_z;
    Vector3d airsim_rpy;

    float fly_roll, fly_pitch, fly_yaw;
    float fly_roll_ned, fly_pitch_ned, fly_yaw_ned;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "airsim_get_image");
    ros::NodeHandle nh_air;
    ros::Time time_current = ros::Time::now();
    double fly_start, fly_finish;
    double flytime;
    int img_i = 0;
    // cv::namedWindow("leftimage", CV_WINDOW_NORMAL);
    // cv::namedWindow("rightimage", CV_WINDOW_NORMAL);

    airsim_image airsim_stereo_img(nh_air);

    airsim_stereo_img.client_connect();
    //fly_start = clock();
    fly_start = ros::Time::now().toSec();
    while (ros::ok())
    {
        airsim_stereo_img.airsim_image_capture();
        //airsim_stereo_img.airsim_Odometry();
        //image_capture();
        time_current = ros::Time::now();
        img_i++;
        ROS_INFO_STREAM("get images:" << img_i);
        //ROS_INFO_STREAM("ros Time now:"<<ros::Time::now()<<"s");

        // fly_finish = ros::Time::now().toSec();
        // flytime = (double)(fly_finish - fly_start) ;/// CLOCKS_PER_SEC;
        // ROS_INFO_STREAM("running time:"<<flytime <<"s");
    }

    return 0;
}
