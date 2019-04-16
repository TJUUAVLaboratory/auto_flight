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

  private:
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
    traj_planning.airsim_init();                             //飞机初始化
    traj_planning.airsim_takeoff(map_world.current_octree_); //飞机起飞(需要一个地图，里面可以没有点)
}

void flying_test::slam()
{
    int i = 1;
    double time1 = 0.0, time2 = 0.0, time3 = 0.0, time4 = 0.0, time5 = 0.0, time6 = 0.0, time7 = 0.0, time8 = 0.0;
    chrono::steady_clock::time_point t1, start_time, finish_time;
    chrono::duration<double> time_used;
    //clock_t t1;
    start_time = chrono::steady_clock::now();
    while (ros::ok()) //&&!traj_planning.arrive_destination(traj_planning.get_trans_msg_a(),traj_planning.get_destinate_coor())
    {
        hitPointsCamera.clear();
        hitPointsPixel.clear();
        t1 = chrono::steady_clock::now();

        // imp.LaplacianPro();
        // time1 = (time1 + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() * 1000);
        // t1 = chrono::steady_clock::now();

        imp.HitPoints(hitPointsCamera, hitPointsPixel);
        time2 = (time2 + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() * 1000);
        t1 = chrono::steady_clock::now();

        framePro.visualizaFrame(imp.imgLeft, hitPointsPixel, imp.blockSize);
        time3 = (time3 + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() * 1000);
        t1 = chrono::steady_clock::now();

        framePro.pixelToCamera(hitPointsCamera);
        time4 = (time4 + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() * 1000);
        t1 = chrono::steady_clock::now();

        framePro.cameraToWorld();
        time5 = (time5 + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() * 1000);
        t1 = chrono::steady_clock::now();

        //boost::mutex::scoped_lock lock( mutex );

        map_world.InsertPointsIntoOctree(framePro.hitPointsWorld);
        time6 = (time6 + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() * 1000);
        t1 = chrono::steady_clock::now();

        framePro.flashFrame();
        map_world.RemoveOldPoints(ros::Time::now(), framePro.get_current_position());
        time7 = (time7 + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() * 1000);
        t1 = chrono::steady_clock::now();

        if (traj_planning.airsim_arrive_destination())
        {
            break;
        }
        
        traj_planning.airsim_traj_plan(map_world.current_octree_);
        time8 = (time8 + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() * 1000);

        ROS_INFO_STREAM("\ntime of laplacian:             " << time1 << "ms"
                                                            << "\ntime of extract point:         " << time2 << "ms"
                                                            << "\ntime of visualizaFrame:        " << time3 << "ms"
                                                            << "\ntime of pixelToCamera:         " << time4 << "ms"
                                                            << "\ntime of cameraToWorld:         " << time5 << "ms"
                                                            << "\ntime of InsertPointsIntoOctree:" << time6 << "ms"
                                                            << "\ntime of flashFrame:            " << time7 << "ms"
                                                            << "\ntime of traj_plan:             " << time8 << "ms"
                                                            << "\n"
                                                            << i);

        ros::spinOnce();

        i++;

        t1 = chrono::steady_clock::now();
    }

    //airsim降落
    traj_planning.airsim_land();

    finish_time = chrono::steady_clock::now();
    time_used =
        chrono::duration_cast<chrono::duration<double>>(finish_time - start_time);
    cout << "总时间：" << time_used.count() * 1000 << "ms" << endl;
}

void flying_test::path_planning()
{
    while (ros::ok())
    {
        //boost::mutex::scoped_lock lock(mutex);

        traj_planning.airsim_traj_plan(map_world.current_octree_);
        if (traj_planning.airsim_arrive_destination())
        {
            break;
        }
    }
}

void flying_test::path_executing()
{
    while (ros::ok())
    {
        traj_planning.execute_waypoints();
        if (traj_planning.airsim_arrive_destination())
        {
            break;
        }
    }
}

void flying_test::muti_thread()
{
    thread thread1(&flying_test::slam, this);
    //thread thread2(&flying_test::path_planning, this);
    thread thread3(&flying_test::path_executing, this);

    thread1.join();
    //thread2.join();
    thread3.join();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_flying_test");
    flying_test airsim_flying_test;
    airsim_flying_test.muti_thread();
    ROS_INFO_STREAM("finish flying!");
    
    return 0;
}
