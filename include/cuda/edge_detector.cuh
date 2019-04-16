#pragma once
#ifndef EDGE_DETECTOR_H
#define EDGE_DETECTOR_H

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <cuda.h>
#include <device_functions.h>

#include <chrono>
#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

using namespace std;
using namespace cv;


struct CudaException : public std::exception
{
  CudaException(const std::string& what, cudaError err)
    : what_(what), err_(err) {}
  virtual ~CudaException() throw() {}
  virtual const char* what() const throw()
  {
    std::stringstream description;
    description << "CudaException: " << what_ << std::endl;
    if(err_ != cudaSuccess)
    {
      description << "cudaError code: " << cudaGetErrorString(err_);
      description << " (" << err_ << ")" << std::endl;
    }
    return description.str().c_str();
  }
  std::string what_;
  cudaError err_;
};

/*
  * Try accessing GPU with pointcloud
  * */
void laplacian_device_gpu(Mat &srcImg, Mat &dstImg, int imgHeight, int imgWidth);

void laplacian_uma_gpu(unsigned char *srcImg, unsigned char *lapImg, Mat &dstImg, int imgHeight, int imgWidth);

#endif
