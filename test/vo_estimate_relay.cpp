#include <ros/ros.h>
#include <std_msgs/String.h>
#include <flight/ImgPro.h>
#include <flight/Frame.h>
#include <flight/StereoMap.h>
#include "ros_traj_plan/searchdistance.h"
#include "ros_traj_plan/uavtrans.h"
#include <sensor_msgs/Imu.h>

class vo_estimate_relay
{
  public:
    vo_estimate_relay();
    ~vo_estimate_relay()
    {
    }
    void getOdom(const nav_msgs::Odometry &msg);
    void getimu(const sensor_msgs::Imu &msg);
    bool imu_flag;

  private:
    ros::NodeHandle nh;
    ros::Subscriber vision_position_sub;
    ros::Subscriber imu_sub;
    ros::Publisher vision_position_pub;

    geometry_msgs::PoseStamped vision_pose;

    geometry_msgs::Quaternion imu_quat;
    Eigen::Quaterniond imu_quat_eigen;
    Eigen::Affine3d pose_T;
    Eigen::Vector3d vision_point_eigen;

    /* data */
};

vo_estimate_relay::vo_estimate_relay()
{
    vision_position_sub = nh.subscribe("/zed/odom", 1, &vo_estimate_relay::getOdom, this);
    imu_sub = nh.subscribe("/mavros/imu/data", 1, &vo_estimate_relay::getimu, this);
    vision_position_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    vision_pose.header.frame_id = "map";
    imu_flag = false;
    pose_T = Eigen::Affine3d::Identity();
}

void vo_estimate_relay::getOdom(const nav_msgs::Odometry &msg)
{
    vision_pose.pose = msg.pose.pose;

    if (imu_flag == true)
    {
        vision_point_eigen = Eigen::Vector3d(vision_pose.pose.position.x, vision_pose.pose.position.y, vision_pose.pose.position.z);

        vision_point_eigen = pose_T * vision_point_eigen;

        vision_pose.pose.position.x = vision_point_eigen(0, 0);
        vision_pose.pose.position.y = vision_point_eigen(1, 0);
        vision_pose.pose.position.z = vision_point_eigen(2, 0);

        vision_pose.header.stamp = ros::Time::now();

        vision_position_pub.publish(vision_pose);
    }
}

void vo_estimate_relay::getimu(const sensor_msgs::Imu &msg)
{
    if (imu_flag == false)
    {
        imu_quat.w = msg.orientation.w;
        imu_quat.x = msg.orientation.x;
        imu_quat.y = msg.orientation.y;
        imu_quat.z = msg.orientation.z;

        imu_quat_eigen = Eigen::Quaterniond(imu_quat.w, imu_quat.x, imu_quat.y, imu_quat.z).normalized();
        pose_T.rotate(imu_quat_eigen);

        // cout << imu_quat_eigen.coeffs().transpose() << endl;
        cout << "Transform matrix = \n"
             << pose_T.matrix() << endl;
        imu_flag = true;

        float mavros_yaw = tf::getYaw(imu_quat) * 180 / 3.14;
        ROS_INFO_STREAM("mavros yaw: " << mavros_yaw << " d\n");
    }

    imu_quat = msg.orientation;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vo_estimate_relay");

    vo_estimate_relay vo_estimate_relay_imp;

    ROS_INFO_STREAM("starting vo_estimate_relay");

    ros::spin();

    return 0;
}