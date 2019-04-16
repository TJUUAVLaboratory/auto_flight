#include <ros/ros.h>
#include "auto_flight/destinate.h"
#include <GeographicLib/Geocentric.hpp>
#include <stdlib.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "set_destination");
    /*
    if (argc!=4)
    {
        ROS_INFO("please input destination coordinate");
        return 1;
    }
   */
    ros::NodeHandle sp_nh;
    ros::NodeHandle private_nh("~");
    ros::ServiceClient des_client = sp_nh.serviceClient<auto_flight::destinate>("set_destination");
    auto_flight::destinate des_srv;

    private_nh.param("destination_x", des_srv.request.x, 250.0);
    private_nh.param("destination_y", des_srv.request.y,-10.0);
    private_nh.param("destination_z", des_srv.request.z, 4.0);

    des_srv.response.des_r = false;
    //des_srv.request.x=atoll(argv[1]);
    //des_srv.request.y=atoll(argv[2]);
    //des_srv.request.z=atoll(argv[3]);

    //std::cout<<"终点：\n"<<des_srv.request.z<<std::endl;

    while (ros::ok())
    {
        if (des_client.call(des_srv) && des_srv.response.des_r == 1)
        {
            ROS_INFO("set destination successfully");

            break;
            // std::cout << "终点：\n"
            //           << des_srv.request.x << std::endl;
            // std::cout << des_srv.request.y << std::endl;
            // std::cout << des_srv.request.z << std::endl;
        }
        // else
        // {
        //     ROS_ERROR("failed to set destination");
        // }

        ROS_INFO("setting destination ");
    }

    return 0;
}
