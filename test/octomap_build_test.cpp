#include <ros/ros.h>
#include <std_msgs/String.h>
#include <flight/ImgPro.h>
#include <flight/Frame.h>
#include <flight/StereoMap.h>
#include "ros_traj_plan/searchdistance.h"
#include "ros_traj_plan/uavtrans.h"
//#include<flight/Config.h>
using namespace std;

class octomap_build_test
{
  public:
    octomap_build_test(string cam_params);
    void slam();
    void path_planning();
    void path_executing();
    void muti_thread();
    void ros_spinonce(const ros::TimerEvent &te);

  private:
    ros::NodeHandle nh;
    ros::Timer spin_timer;
    image_transport::ImageTransport it;
    image_transport::Subscriber subLeftImg;
    image_transport::Subscriber subRightImg;
    image_transport::Subscriber subDepthImg;
    ros::Subscriber odom;

    imgPro imp;
    Frame framePro;
    StereoMap map_world;
    //Uav_trajplannimg traj_planning;
    boost::mutex mutex;

    vector<Point3f> hitPointsCamera;
    vector<Point3i> hitPointsPixel;
};

// subLeftImg(it.subscribe("/zed/left/image_rect_color/compressed", 1, &imgPro::getImgLeft, &imp)),
//                                                             subRightImg(it.subscribe("/zed/right/image_rect_color/compressed", 1, &imgPro::getImgRight, &imp)),
//                                                             subDepthImg(it.subscribe("/zed/depth/depth_registered/compressedDepth", 1, &imgPro::getImgDepth, &imp)),

octomap_build_test::octomap_build_test(string cam_params) : imp(cam_params),
                                                            framePro((imp.matQ)),
                                                            it(nh),
                                                            subLeftImg(it.subscribe("/zed/left/image_rect_color", 1, &imgPro::getImgLeft, &imp)),
                                                            subRightImg(it.subscribe("/zed/right/image_rect_color", 1, &imgPro::getImgRight, &imp)),
                                                            subDepthImg(it.subscribe("/zed/depth/depth_registered", 1, &imgPro::getImgDepth, &imp)),
                                                            odom(nh.subscribe("/zed/odom", 1, &Frame::getOdom, &framePro))
{
    //spin_timer = nh.createTimer(ros::Duration(0.025), &octomap_build_test::ros_spinonce, this);
    framePro.imageResultleft = it.advertise("/auto_flight/obstacle_points_img", 10);
    imp.image_pub_laplacianLeft = it.advertise("/auto_flight//image_laplacianLeft", 1);
    // traj_planning.fly_init();                             //飞机初始化
    // traj_planning.fly_takeoff(map_world.current_octree_); //飞机起飞(需要一个地图，里面可以没有点)
}

void octomap_build_test::ros_spinonce(const ros::TimerEvent &te)
{
    ros::spinOnce();
}

void octomap_build_test::slam()
{

    int i = 1;
    double time1 = 0.0, time2 = 0.0, time3 = 0.0, time4 = 0.0, time5 = 0.0, time6 = 0.0, time7 = 0.0, time8 = 0.0, time9 = 0.0;
    chrono::steady_clock::time_point t1, start_time, finish_time;
    chrono::duration<double> time_used;
    imp.image_num = 0;
    framePro.image_pose_num = 0;
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

        ROS_INFO_STREAM("image_pose_num:"<<framePro.image_pose_num);

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

        map_world.InsertPointsIntoOctree(framePro.hitPointsWorld);
        time6 = (time6 + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() * 1000);
        t1 = chrono::steady_clock::now();

        framePro.flashFrame();
        map_world.RemoveOldPoints(ros::Time::now(), framePro.get_current_position());
        time7 = (time7 + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() * 1000);
        t1 = chrono::steady_clock::now();

        // traj_planning.fly_traj_plan(map_world.current_octree_);

        // if (traj_planning.arrive_destination(traj_planning.get_trans_msg_a(), traj_planning.get_destinate_coor()))
        // {
        //     break;
        // }

        time8 = (time8 + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() * 1000);
        t1 = chrono::steady_clock::now();

        ros::spinOnce();
        time9 = (time9 + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() * 1000);

        ROS_INFO_STREAM("\ntime of laplacian:             " << time1 << "ms"
                                                            << "\ntime of extract point:         " << time2 << "ms"
                                                            << "\ntime of visualizaFrame:        " << time3 << "ms"
                                                            << "\ntime of pixelToCamera:         " << time4 << "ms"
                                                            << "\ntime of cameraToWorld:         " << time5 << "ms"
                                                            << "\ntime of InsertPointsIntoOctree:" << time6 << "ms"
                                                            << "\ntime of flashFrame:            " << time7 << "ms"
                                                            << "\ntime of traj_plan:             " << time8 << "ms"
                                                            << "\ntime of ros_spin :             " << time9 << "ms"
                                                            << "\n"
                                                            << i << "\n");

        i++;

        t1 = chrono::steady_clock::now();
    }

    finish_time = chrono::steady_clock::now();
    time_used =
        chrono::duration_cast<chrono::duration<double>>(finish_time - start_time);
    cout << "总时间：" << time_used.count() * 1000 << "ms" << endl;

    // while (ros::ok())
    // {
    //     traj_planning.pub_waypoint(traj_planning.get_destinate_coor());
    // }
}

void octomap_build_test::path_planning()
{
    while (ros::ok())
    {
    }
}

void octomap_build_test::path_executing()
{
    ros::Rate spin_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        spin_rate.sleep();
    }
}

void octomap_build_test::muti_thread()
{
    thread thread1(&octomap_build_test::slam, this);
    //thread thread2(&octomap_build_test::path_planning, this);
    //thread thread3(&octomap_build_test::path_executing, this);

    thread1.join();
    //thread2.join();
    //thread3.join();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_build_test");

    ros::NodeHandle private_nh("~");
    string cam_params;
    private_nh.param("cam_params", cam_params, std::string("src/auto_flight/default.yaml"));

    octomap_build_test octomap_build_test(cam_params);
    octomap_build_test.muti_thread();
    ROS_INFO_STREAM("finish flying!");
    return 0;
}
