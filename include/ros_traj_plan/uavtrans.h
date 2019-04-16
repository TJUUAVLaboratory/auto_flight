#pragma once
#ifndef UAVTRANS_H
#define UAVTRANS_H

#include <iostream>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <string.h>
#include <cmath>
#include <ctime>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/PositionTarget.h>
#include <thread>
#include <boost/thread/mutex.hpp>

struct uavTrans
{
    double uav_quat[4];     //转换矩阵
    double uav_position[3]; //飞机坐标
};
//typedef struct _uavTrans uavTrans;

void delay_msec(float msec);
std::string goal_getTime();
double NearestNeighbor(float point[3], pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map);
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *InsertPointsIntoOctree(pcl::PointCloud<pcl::PointXYZ> cloud_msg);
uavTrans set_trans_init();
geometry_msgs::Pose set_start_point();

/**
 * board_to_world:
 * @utrans: input rigid body transformation
 * @src: inpput vector
 * @dst: output vector
 *
 * Applies the rigid body transformation to a vector.
 */
void board_to_world(const uavTrans *utrans, const double src[], float dst[3]);
void world_to_board(const uavTrans *utrans, const double src[], float dst[3]);

/**
 * rotation_quat:
 *
 */
void rotation_quat(const double quat[4], const double v[], float result[3]);

/**
 * bot_trans_copy:
 * @dest: output parameter
 * @src: source #uavTrans
 *
 * Makes a copy of a #uavTrans
 */
void bot_trans_copy(uavTrans *dest, const uavTrans *src);

/**
 * converts a rotation from RPY representation (radians) into unit quaternion
 * representation
 *
 * rpy[0] = roll
 * rpy[1] = pitch
 * rpy[2] = yaw
 */
void bot_roll_pitch_yaw_to_quat(const double rpy[3], double q[4]);

/**
 * converts a rotation from unit quaternion representation to RPY
 * representation.  Resulting values are in radians.
 *
 * If any of roll, pitch, or yaw are NULL, then they are not set.
 *
 * rpy[0] = roll
 * rpy[1] = pitch
 * rpy[2] = yaw
 */
void bot_quat_to_roll_pitch_yaw(const double q[4], double rpy[3]);

#endif
