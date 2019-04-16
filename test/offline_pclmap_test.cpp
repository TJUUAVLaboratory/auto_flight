#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <assert.h>
#include <iostream>

main(int argc, char **argv)
{
    ros::init(argc, argv, "offline_pclmap_test");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher pcl_pub =
        nh.advertise<sensor_msgs::PointCloud2>("pcl_output_map", 2);
    ros::Publisher Octomap_pub =
        nh.advertise<octomap_msgs::Octomap>("octomap_tree", 1);
    double map_resolution = 0.3;

    std::string pcl_file;
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *current_octree_;
    octomap::OcTree tree(map_resolution);
    octomap_msgs::Octomap msg_octomap;
    float transit;
    size_t cloud_i = 0;

    // pcl::io::loadPCDFile ("result2.pcd", cloud);
    private_nh.param("pcl_file", pcl_file, std::string("src/auto_flight/mapdata/result3.pcd"));
    //打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcl_file,
                                            cloud) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    //默认就是而二进制块读取转换为模块化的PointCLoud格式里pcl::PointXYZ作为点类型
    //然后打印出来
    std::cout << "Loaded " << cloud.width << "*" << cloud.height
              << " data points from pcd with the following fields: " << std::endl;

    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        // transit = cloud.points[i].z / 400;
        // cloud.points[i].z = 4.0 + cloud.points[i].y / 400;
        // cloud.points[i].y = cloud.points[i].x / 400 + 2.0;
        // cloud.points[i].x = transit + 13.0;

        transit = cloud.points[i].z / 1000;
        cloud.points[i].z = cloud.points[i].x / 1000 + 5.0;
        cloud.points[i].x = 4.0 + cloud.points[i].y / 1000 + 8.0;
        cloud.points[i].y = transit - 30;
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
    statFilter.setInputCloud(cloud.makeShared());
    statFilter.setMeanK(5);
    statFilter.setStddevMulThresh(0.5);
    statFilter.filter(cloud_filtered);

    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(cloud_filtered.makeShared());
    voxelSampler.setLeafSize(0.05f, 0.05f, 0.05f);
    voxelSampler.filter(cloud_downsampled);

    cloud_downsampled.swap(cloud);

    std::cout << "cloud_filtered: " << cloud.width << "*" << cloud.height
              << std::endl;

    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map";

    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        tree.updateNode(octomap::point3d(cloud.points[i].x, cloud.points[i].y,
                                         cloud.points[i].z),
                        true);
    }

    // 更新octomap
    tree.updateInnerOccupancy();

    // prepare octomap msg
    if (octomap_msgs::binaryMapToMsg(tree, msg_octomap))
    {
        ROS_INFO("\nTransfer to octomap_msgs successfully!");
    }
    msg_octomap.binary = 1;
    msg_octomap.id = tree.getTreeType();
    msg_octomap.resolution = map_resolution;
    msg_octomap.header.frame_id = "/map";
    msg_octomap.header.stamp = ros::Time::now();

    ROS_INFO("Octomap updated in RVIZ");

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        Octomap_pub.publish(msg_octomap);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
