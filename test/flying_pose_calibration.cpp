#include <ros/ros.h>
#include <std_msgs/String.h>
#include <flight/ImgPro.h>
#include <flight/Frame.h>
#include <flight/StereoMap.h>
#include "ros_traj_plan/searchdistance.h"
#include "ros_traj_plan/uavtrans.h"
//#include<flight/Config.h>
using namespace std;

class flying_test
{
  public:
    flying_test();
    void slam();
    void path_planning();
    void path_executing();
    void muti_thread();

    //private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber subLeftImg;
    image_transport::Subscriber subRightImg;
    ros::Subscriber odom;

    imgPro imp;
    Frame framePro;
    StereoMap map_world;
    Uav_trajplannimg traj_planning;
    boost::mutex mutex;

    vector<Point3f> hitPointsCamera;
    vector<Point3i> hitPointsPixel;
};

flying_test::flying_test() : imp("src/auto_flight/default.yaml"),
                             framePro((imp.matQ)),
                             it(nh),
                             subLeftImg(it.subscribe("/zed/left/image_rect_color", 1, &imgPro::getImgLeft, &imp)),
                             subRightImg(it.subscribe("/zed/right/image_rect_color", 1, &imgPro::getImgRight, &imp)),
                             odom(nh.subscribe("/zed/odom", 1, &Frame::getOdom, &framePro))
{
    framePro.imageResultleft = it.advertise("/auto_flight/obstacle_points_img", 10);
    imp.image_pub_laplacianLeft = it.advertise("/auto_flight//image_laplacianLeft", 1);
    traj_planning.fly_init(); //飞机初始化
}

void flying_test::slam()
{
}

void flying_test::path_planning()
{
    while (ros::ok())
    {
    }
}

void flying_test::path_executing()
{
    while (ros::ok())
    {
    }
}

void flying_test::muti_thread()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flying_pose_calibration");
    flying_test mavros_flying_test;
    ros::NodeHandle nh1;

    ros::Publisher vision_pub = nh1.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    geometry_msgs::PoseStamped pose;
    nav_msgs::Odometry odom1;
    geometry_msgs::PoseStamped mavros_pose, camera_pose;
    double mavros_yaw;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;

    int i = 200;
    while (i > 0)
    {
        mavros_flying_test.traj_planning.fly_position_calibration(pose);

        mavros_pose = mavros_flying_test.traj_planning.get_trans_msg_a();
        //vision_pub.publish(mavros_pose);
        ROS_INFO_STREAM("mavros pose: x:" << mavros_pose.pose.position.x << "  y:" << mavros_pose.pose.position.y << "  z:" << mavros_pose.pose.position.z);
        ROS_INFO_STREAM(setprecision(6) << fixed << "mavros orientation  x:" << mavros_pose.pose.orientation.x << " y:," << mavros_pose.pose.orientation.y << " z:," << mavros_pose.pose.orientation.z << " w:," << mavros_pose.pose.orientation.w);

        mavros_yaw = tf::getYaw(mavros_pose.pose.orientation) * 180 / 3.14;
        ROS_INFO_STREAM("mavros yaw: " << mavros_yaw << " d\n");

        ROS_INFO_STREAM("i=:" << i);

        ros::spinOnce();
        rate.sleep();
        --i;
    }

    pose.pose.position.x = 5;
    pose.pose.position.y = 5;
    pose.pose.position.z = 5;

    // odom1.pose.pose.position.x = 4;
    // odom1.pose.pose.position.y = 0;
    // odom1.pose.pose.position.z = 4;

    pose.header.frame_id = "map";
    //pose.child_frame_id = "base_link";

    // odom1.header.frame_id = "local_origin_ned";
    // odom1.child_frame_id = "fcu_frd";

    i = 200;

    while (i > 0)
    {
        mavros_flying_test.traj_planning.fly_position_calibration(pose);
        //vision_pub.publish(mavros_pose);

        mavros_pose = mavros_flying_test.traj_planning.get_trans_msg_a();
        ROS_INFO_STREAM("mavros pose: x:" << mavros_pose.pose.position.x << "  y:" << mavros_pose.pose.position.y << "  z:" << mavros_pose.pose.position.z);
        ROS_INFO_STREAM(setprecision(6) << fixed << "mavros orientation  x:" << mavros_pose.pose.orientation.x << " y:," << mavros_pose.pose.orientation.y << " z:," << mavros_pose.pose.orientation.z << " w:," << mavros_pose.pose.orientation.w);

        mavros_yaw = tf::getYaw(mavros_pose.pose.orientation) * 180 / 3.14;
        ROS_INFO_STREAM("mavros yaw: " << mavros_yaw << " d\n");
        
         ROS_INFO_STREAM("i=:" << i);
        ros::spinOnce();
        rate.sleep();

        --i;
    }

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;

    pose.header.frame_id = "map";

    i = 200;

    while (i > 0)
    {
        mavros_flying_test.traj_planning.fly_position_calibration(pose);
        //vision_pub.publish(mavros_pose);

        mavros_pose = mavros_flying_test.traj_planning.get_trans_msg_a();
        ROS_INFO_STREAM("mavros pose: x:" << mavros_pose.pose.position.x << "  y:" << mavros_pose.pose.position.y << "  z:" << mavros_pose.pose.position.z);
        ROS_INFO_STREAM(setprecision(6) << fixed << "mavros orientation  x:" << mavros_pose.pose.orientation.x << " y:," << mavros_pose.pose.orientation.y << " z:," << mavros_pose.pose.orientation.z << " w:," << mavros_pose.pose.orientation.w);

        mavros_yaw = tf::getYaw(mavros_pose.pose.orientation) * 180 / 3.14;
        ROS_INFO_STREAM("mavros yaw: " << mavros_yaw << " d\n");

        ROS_INFO_STREAM("i=:" << i);

        ros::spinOnce();
        rate.sleep();

        --i;
    }

    mavros_flying_test.traj_planning.mavros_auto_land();

    return 0;
}