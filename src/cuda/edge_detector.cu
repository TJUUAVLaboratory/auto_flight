#include "cuda/edge_detector.cuh"
//#include "flight/ImgPro.h"


// laplacian算子边缘检测核函数
__global__ void laplacianincuda(unsigned char *dataIn, unsigned char *dataOut,
                            int imgHeight, int imgWidth) {
  int xIndex = threadIdx.x + blockIdx.x * blockDim.x;
  int yIndex = threadIdx.y + blockIdx.y * blockDim.y;
  int index = yIndex * imgWidth + xIndex;
  int Gx = 0;
  int Gy = 0;
  int gl;

  if (xIndex > 0 && xIndex < imgWidth - 1 && yIndex > 0 &&
      yIndex < imgHeight - 1) {


  //     Gx = dataIn[(yIndex - 1) * imgWidth + xIndex + 1] +
  //        2 * dataIn[yIndex * imgWidth + xIndex + 1] +
  //        dataIn[(yIndex + 1) * imgWidth + xIndex + 1] -
  //        (dataIn[(yIndex - 1) * imgWidth + xIndex - 1] +
  //         2 * dataIn[yIndex * imgWidth + xIndex - 1] +
  //         dataIn[(yIndex + 1) * imgWidth + xIndex - 1]);
  //   Gy = dataIn[(yIndex - 1) * imgWidth + xIndex - 1] +
  //        2 * dataIn[(yIndex - 1) * imgWidth + xIndex] +
  //        dataIn[(yIndex - 1) * imgWidth + xIndex + 1] -
  //        (dataIn[(yIndex + 1) * imgWidth + xIndex - 1] +
  //         2 * dataIn[(yIndex + 1) * imgWidth + xIndex] +
  //         dataIn[(yIndex + 1) * imgWidth + xIndex + 1]);
  //  dataOut[index] = (abs(Gx) + abs(Gy)) / 4;

  //laplacian
     
     gl = 4 * dataIn[yIndex * imgWidth + xIndex] -
         dataIn[(yIndex - 1) * imgWidth + xIndex] -
         dataIn[yIndex * imgWidth + xIndex - 1] -
         dataIn[yIndex * imgWidth + xIndex + 1] -
         dataIn[(yIndex + 1) * imgWidth + xIndex];
     dataOut[index] = abs(gl) * 2;
     
  }
}

void laplacian_device_gpu(Mat &srcImg, Mat &dstImg, int imgHeight, int imgWidth)
{
    // CUDA实现后的传回的图像
    //创建事件，启动定时
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start, 0);
  

  //创建GPU内存
  unsigned char *d_in;
  unsigned char *d_out;

  cudaMalloc((void **)&d_in, imgHeight * imgWidth * sizeof(unsigned char));
  cudaMalloc((void **)&d_out, imgHeight * imgWidth * sizeof(unsigned char));

  cudaMemcpy(d_in, srcImg.data, imgHeight * imgWidth * sizeof(unsigned char),
             cudaMemcpyHostToDevice);

  dim3 threadsPerBlock(16, 16);
  dim3 blocksPerGrid((imgWidth + threadsPerBlock.x - 1) / threadsPerBlock.x,
                     (imgHeight + threadsPerBlock.y - 1) / threadsPerBlock.y);


  //调用核函数
  laplacianincuda<<<blocksPerGrid, threadsPerBlock>>>(d_in, d_out, imgHeight,
                                                  imgWidth);

  //将图像传回GPU
  cudaMemcpy(dstImg.data, d_out, imgHeight * imgWidth * sizeof(unsigned char),
             cudaMemcpyDeviceToHost);

  cudaEventRecord(stop, 0);
  cudaEventSynchronize(start); //事件同步语句
  cudaEventSynchronize(stop);  //事件同步语句
  //计算CUDA耗时，并显示时间
  float GPUTime_Global = 0;
  cudaEventElapsedTime(&GPUTime_Global, start, stop);
  cout << "GPU:laplacian边缘提取时间:" << GPUTime_Global << "ms" << endl;

  //释放GPU内存
  cudaFree(d_in);
  cudaFree(d_out);
}

void laplacian_uma_gpu(unsigned char *srcImg, unsigned char *lapImg, Mat &dstImg, int imgHeight, int imgWidth)
{
    // cudaEvent_t start, stop;
    // cudaEventCreate(&start);
    // cudaEventCreate(&stop);
    // cudaEventRecord(start, 0);
    
    // unsigned char *h_src,*h_dst;
    // //unsigned char *d_src,*d_dst;
  
    // cudaMallocManaged(&h_src, imgHeight * imgWidth * sizeof(unsigned char));
    // cudaMallocManaged(&h_dst, imgHeight * imgWidth * sizeof(unsigned char));
  
    // memcpy((unsigned char*)h_src,(unsigned char*)srcImg.data,imgHeight * imgWidth * sizeof(unsigned char));

        // // 使用 std::chrono 来给算法计时
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  
    dim3 threadsPerBlock(16, 16);
    dim3 blocksPerGrid((imgWidth + threadsPerBlock.x - 1) / threadsPerBlock.x,
                       (imgHeight + threadsPerBlock.y - 1) / threadsPerBlock.y);
  
    //调用核函数
    laplacianincuda<<<blocksPerGrid, threadsPerBlock>>>(srcImg, lapImg, imgHeight,
                                                    imgWidth);
    
    cudaError_t error = cudaGetLastError();
    printf("CUDA error: %s\n", cudaGetErrorString(error));
     // CUDA实现后的传回的图像
     //Mat dstImg(imgHeight, imgWidth, CV_8UC1, Scalar(0));                                                
     
     cudaDeviceSynchronize();

     memcpy((unsigned char*)dstImg.data,(unsigned char*)lapImg,imgHeight * imgWidth * sizeof(unsigned char));

     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
     chrono::duration<double> time_used =
         chrono::duration_cast<chrono::duration<double>>(t2 - t1);
     ROS_INFO_STREAM("GPU:laplacian time(chrono):" << time_used.count() * 1000 << "ms");

    // cudaEventRecord(stop, 0);
    // cudaEventSynchronize(start); //事件同步语句
    // cudaEventSynchronize(stop);  //事件同步语句
    // //计算CUDA耗时，并显示时间
    // float GPUTime_Global = 0;
    // cudaEventElapsedTime(&GPUTime_Global, start, stop);
    // cout << "GPU:laplacian边缘提取时间:" << GPUTime_Global << "ms" << endl;
  
  
    // cv::namedWindow("laplacian_uma_cuda", 0);
    // cv::imshow("laplacian_uma_cuda", dstImg);
    // cv::waitKey(0);
  
    //释放GPU内存
    // cudaFree(h_src);
    // cudaFree(h_dst);
}

// void img_process_gpu(imgPro *imp_)
// {

// }
