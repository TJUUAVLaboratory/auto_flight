#pragma once
#ifndef IMG_PROCESS_GPU_H
#define IMG_PROCESS_GPU_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "cuda.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <std_msgs/Float32MultiArray.h>
#include "cuda/edge_detector.cuh"

static void HandleError( cudaError_t err,
                         const char *file,
                         int line ) {
    if (err != cudaSuccess) {
        printf( "%s in %s at line %d\n", cudaGetErrorString( err ),
                file, line );
        exit( EXIT_FAILURE );
    }
}
#define HANDLE_ERROR( err ) (HandleError( err, __FILE__, __LINE__ ))

struct image_process
{

    int INVARIANCE_CHECK_HORZ_OFFSET_MIN, INVARIANCE_CHECK_HORZ_OFFSET_MAX, INVARIANCE_CHECK_VERT_OFFSET_INCREMENT;
    int blockSize, INVARIANCE_CHECK_VERT_OFFSET_MIN, INVARIANCE_CHECK_VERT_OFFSET_MAX, horizontalInvarianceMultiplier;
    int sadThreshold;
    int sad;
    int zero_disparity, sobelLimit, disparity, true_disparity;
    bool check_horizontal_invariance;

    float up_error, down_error;
    float depth2disp_ratio;

    int *pixel_points;
    float *camera_points;
    int point_num;
    int sum_of_point_num;
    int sum_of_point;

    unsigned char *src_img1, *src_img2;
    unsigned char *lapla_img1, *lapla_img2;
    float *depth_img, *zed_depth_img ;

    int imgWidth, imgHeight;

    image_process *dev_ptr;

    image_process(int imgWidth_, int imgHeight_)
    {
        imgWidth = imgWidth_;
        imgHeight = imgHeight_;

        cudaMallocManaged(&src_img1, imgHeight * imgWidth * sizeof(unsigned char));
        cudaMallocManaged(&src_img2, imgHeight * imgWidth * sizeof(unsigned char));
        cudaMallocManaged(&lapla_img1, imgHeight * imgWidth * sizeof(unsigned char));
        cudaMallocManaged(&lapla_img2, imgHeight * imgWidth * sizeof(unsigned char));

        cudaMallocManaged(&depth_img, imgHeight * imgWidth * sizeof(float));
        cudaMallocManaged(&zed_depth_img, imgHeight * imgWidth * sizeof(float));

        cudaMallocManaged(&pixel_points, imgHeight * imgWidth * sizeof(int));
        cudaMallocManaged(&camera_points, imgHeight * imgWidth * sizeof(float));
        cudaMemset(pixel_points, 0, imgHeight * imgWidth * sizeof(int));
        cudaMemset(camera_points, 0, imgHeight * imgWidth * sizeof(float));

        //memcpy((unsigned char *)h_src, (unsigned char *)srcImg.data, imgHeight * imgWidth * sizeof(unsigned char));

        // cudaError cuda_err = cudaMallocManaged(&dev_ptr, sizeof(*this));
        // if (cuda_err != cudaSuccess)
        //     throw CudaException("DeviceImage: cannot copy image parameters to device memory.", cuda_err);
        

        // memcpy(src_img1,this->src_img1,sizeof(*this->src_img1));
        // memcpy(src_img2,this->src_img2,sizeof(*this->src_img2));
        // memcpy(lapla_img1,this->lapla_img1,sizeof(*this->lapla_img1));
        // memcpy(lapla_img2,this->lapla_img2,sizeof(*this->lapla_img2));
        // memcpy(depth_img,this->depth_img,sizeof(*this->depth_img));

        // memcpy(pixel_points,this->pixel_points,sizeof(*this->pixel_points));
        // memcpy(camera_points,this->camera_points,sizeof(*this->camera_points));

        // memcpy(
        //     dev_ptr,
        //     this,
        //     sizeof(*this));
        
        //cout << "init image_process GPU memory" << endl;
     }

    void param_memocpy()
    {
        // memcpy(src_img1,this->src_img1,sizeof(*this->src_img1));
        // memcpy(src_img2,this->src_img2,sizeof(*this->src_img2));
        // memcpy(lapla_img1,this->lapla_img1,sizeof(*this->lapla_img1));
        // memcpy(lapla_img2,this->lapla_img2,sizeof(*this->lapla_img2));
        // memcpy(depth_img,this->depth_img,sizeof(*this->depth_img));

        // memcpy(pixel_points,this->pixel_points,sizeof(*this->pixel_points));
        // memcpy(camera_points,this->camera_points,sizeof(*this->camera_points));

         memcpy(
            dev_ptr,
            this,
            sizeof(*this));
            //cudaMemcpyHostToDevice);

        cout << "copy image_process GPU memory" << endl;
        point_num = -1;
    }

    void set_raw_image_data(Mat &leftimg, Mat &rightimg)
    {
        memcpy(src_img1, leftimg.data, imgHeight * imgWidth * sizeof(unsigned char));
                 //  cudaMemcpyHostToDevice);

        memcpy(src_img2, rightimg.data, imgHeight * imgWidth * sizeof(unsigned char));
                 //  cudaMemcpyHostToDevice);

        ROS_INFO_STREAM("GPU:copy image dada!");
    }
    void set_lapla_image_data(Mat &leftimg, Mat &rightimg)
    {
    }
    void set_depth_image_data(std_msgs::Float32MultiArray &Disparity_array)
    {
        memcpy((float *)depth_img, (float *)&Disparity_array.data[0], imgHeight * imgWidth * sizeof(float));
                //   cudaMemcpyHostToDevice);
        //cout << "GPU:copy depth dada!" << endl;

        ROS_INFO_STREAM("GPU:copy depth dada!");
    }

    void get_pixel_points(vector<Point3i> &pointVector2d)
    {
        if (point_num >= 0)
        {
            for (int i = 0; i < point_num + 1; i++)
            {
                if (pixel_points[i * 3 + 2] == 0 || pixel_points[i * 3] == 0 || pixel_points[i * 3 + 1] == 0)
                    continue;

                pointVector2d.push_back(Point3i(pixel_points[i * 3], pixel_points[i * 3 + 1],
                                                pixel_points[i * 3 + 2]));
            }
        }
    }

    void get_camera_points(vector<Point3f> &localHitPoints)
    {
        if (point_num >= 0)
        {
            for (int i = 0; i < point_num + 1; i++)
            {
                if (camera_points[i * 3 + 2] == 0.0 || camera_points[i * 3] == 0.0 || camera_points[i * 3 + 1] == 0.0)
                    continue;
                localHitPoints.push_back(Point3f(camera_points[i * 3], camera_points[i * 3 + 1],
                                                 camera_points[i * 3 + 2]));
            }
        }
    }
};

__global__ void image_process_cuda(image_process *imp_, int imgHeight, int imgWidth);

__global__ void image_process_sad_cuda(image_process *imp_, int imgHeight, int imgWidth);

void image_process_gpu(image_process *imp_);

__device__ int getSAD(image_process *imp_, int pxX, int pxY, int imgHeight, int imgWidth);

__device__ bool checkHorizontalInvariance(image_process *imp_, int pxX, int pxY, int imgHeight, int imgWidth);

__device__ bool check_Disparity(image_process *imp_, int pxX, int pxY, int imgHeight, int imgWidth);

#endif