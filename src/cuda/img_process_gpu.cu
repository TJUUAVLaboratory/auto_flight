
#include "cuda/img_process_gpu.cuh"


__device__ __managed__ int *point_index;


__global__ void image_process_cuda(image_process *imp_, int imgHeight, int imgWidth)
{
    int thread_xIndex = threadIdx.x + blockIdx.x * blockDim.x;
    int thread_yIndex = threadIdx.y + blockIdx.y * blockDim.y;
    int thread_index = thread_yIndex * imgWidth + thread_xIndex;

    int xIndex = (threadIdx.x + blockIdx.x * blockDim.x) * imp_->blockSize - imp_->disparity;
    int yIndex = (threadIdx.y + blockIdx.y * blockDim.y) * imp_->blockSize;
    int index = yIndex * imgWidth + xIndex;

    int gl, gr;

    if (thread_xIndex > 0 && thread_xIndex < imgWidth - 1 && thread_yIndex > 0 &&
        thread_yIndex < imgHeight - 1)
    {
        gl = 4 * imp_->src_img1[thread_yIndex * imgWidth + thread_xIndex] -
             imp_->src_img1[(thread_yIndex - 1) * imgWidth + thread_xIndex] -
             imp_->src_img1[thread_yIndex * imgWidth + thread_xIndex - 1] -
             imp_->src_img1[thread_yIndex * imgWidth + thread_xIndex + 1] -
             imp_->src_img1[(thread_yIndex + 1) * imgWidth + thread_xIndex];

        gl = gl * 2;

        if (gl <= 0)
        {
            gl = abs(gl);
        }
        if (gl > 255)
        {
            gl = 255;
        }
        imp_->lapla_img1[thread_index] = gl;

        gr = 4 * imp_->src_img2[thread_yIndex * imgWidth + thread_xIndex] -
             imp_->src_img2[(thread_yIndex - 1) * imgWidth + thread_xIndex] -
             imp_->src_img2[thread_yIndex * imgWidth + thread_xIndex - 1] -
             imp_->src_img2[thread_yIndex * imgWidth + thread_xIndex + 1] -
             imp_->src_img2[(thread_yIndex + 1) * imgWidth + thread_xIndex];

        gr = gr * 2;

        if (gr <= 0)
        {
            gl = abs(gr);
        }
        if (gr > 255)
        {
            gr = 255;
        }
        imp_->lapla_img2[thread_index] = gr;
    }
    // if(thread_index%10000 == 0)
    // {
    //     printf("thread_index=%d\n", thread_index);
    // }
    __syncthreads();

    if (xIndex > -1 * imp_->disparity && xIndex < imgWidth - 1 - imp_->blockSize && yIndex > 0 &&
        yIndex < imgHeight - 1 - imp_->blockSize)
    {

        int sad = getSAD(imp_, xIndex, yIndex, imgHeight, imgWidth);

        if (sad < imp_->sadThreshold && sad > 0)
        {
            int dj = xIndex + imp_->blockSize / 2;
            int di = yIndex + imp_->blockSize / 2;
            //printf("row = %d, col = %d, sad = %d\n",di, dj, sad);

            //bool if_horizon = true;
            //if_horizon = checkHorizontalInvariance(imp_, xIndex, yIndex, imgHeight, imgWidth);
            // if (imp_->check_horizontal_invariance && if_horizon  == false)
            // {
            //bool if_true_disp = check_Disparity(imp_, dj, di, imgHeight, imgWidth);

            float true_depth = imp_->zed_depth_img[di * imgWidth + dj - 1];

            if (true_depth < 6.0 && true_depth > 3.0)
            {
                float true_disparity = imp_->depth2disp_ratio / true_depth; //matQ.at<double>(2, 3) / (-1.0 * matQ.at<double>(3, 2) * true_depth * 1000);
                int point_i = atomicAdd(&(imp_->point_num), 1) + 1;

                // __syncthreads();

                imp_->camera_points[point_i * 3] = dj;
                imp_->camera_points[point_i * 3 + 1] = di;
                imp_->camera_points[point_i * 3 + 2] = true_disparity;

                imp_->pixel_points[point_i * 3] = xIndex;
                imp_->pixel_points[point_i * 3 + 1] = yIndex;
                imp_->pixel_points[point_i * 3 + 2] = sad;

                int startX = xIndex;
                int startY = yIndex;
                int endX = xIndex + imp_->blockSize - 1;
                int endY = yIndex + imp_->blockSize - 1;
            
                int leftVal = 0;
            
                for (int i = startY; i <= endY; i++)
                {
                    for (int j = startX; j <= endX; j++)
                    {
                        unsigned char sL = imp_->lapla_img1[i * imgWidth + j];
                        leftVal += sL;
                    }
                }
                //printf("point_num = %d\n",imp_->point_num );
                printf("point_num = %d, row = %d, col = %d, true_depth = %f, sad = %d, lapla_l = %d\n",point_i,di, dj, true_depth,imp_->pixel_points[point_i * 3 + 2], leftVal);
            }

            // float true_disparity = imp_->depth_img[di * imgWidth + dj - 1] * imgWidth;
            // float dis_error = true_disparity - (-1.0f * imp_->disparity);

            // if (dis_error >= imp_->down_error) //&& dis_error <= up_error
            // {
            //     int point_i = atomicAdd(&(imp_->point_num), 1) + 1;

            //    // __syncthreads();

            //     imp_->camera_points[point_i * 3]     = dj;
            //     imp_->camera_points[point_i * 3 + 1] = di;
            //     imp_->camera_points[point_i * 3 + 2] = true_disparity;

            //     imp_->pixel_points[point_i * 3    ] = xIndex;
            //     imp_->pixel_points[point_i * 3 + 1] = yIndex;
            //     imp_->pixel_points[point_i * 3 + 2] = sad;
            //     //printf("point_num = %d\n",imp_->point_num );
            //     //printf("point_num = %d, row = %d, col = %d, disp = %f, sad = %d\n",point_i,di, dj, imp_->camera_points[point_i * 3 + 2],imp_->pixel_points[point_i * 3 + 2]);
            // }
            //}
        }
    }
}

__global__ void image_process_sad_cuda(image_process *imp_, int imgHeight, int imgWidth)
{
    int xIndex = (threadIdx.x + blockIdx.x * blockDim.x) * imp_->blockSize - imp_->disparity;
    int yIndex = (threadIdx.y + blockIdx.y * blockDim.y) * imp_->blockSize;
    int index = yIndex * imgWidth + xIndex;

    int thread_xIndex = (threadIdx.x + blockIdx.x * blockDim.x) * imp_->blockSize;
    int thread_yIndex = (threadIdx.y + blockIdx.y * blockDim.y) * imp_->blockSize;
    int thread_width = blockDim.x  * gridDim.x;
    int thread_index;// = thread_yIndex * thread_width + thread_xIndex;

    int gl, gr;

    if (thread_xIndex > 0 && thread_xIndex < imgWidth - imp_->blockSize - 1 && thread_yIndex > 0 &&
        thread_yIndex < imgHeight - imp_->blockSize - 1)
    {
        for(int i = 0; i < imp_->blockSize; i++)
            for(int j = 0; j < imp_->blockSize; j++)
            {
                thread_index = (thread_yIndex + i) * imgWidth + (thread_xIndex + j);

                gl = 4 * imp_->src_img1[(thread_yIndex + i) * imgWidth + (thread_xIndex + j)] -
             imp_->src_img1[((thread_yIndex + i) - 1) * imgWidth + (thread_xIndex + j)] -
             imp_->src_img1[(thread_yIndex + i) * imgWidth + (thread_xIndex + j) - 1] -
             imp_->src_img1[(thread_yIndex + i) * imgWidth + (thread_xIndex + j) + 1] -
             imp_->src_img1[((thread_yIndex + i) + 1) * imgWidth + (thread_xIndex + j)];
            imp_->lapla_img1[thread_index] = abs(gl);

            gr = 4 * imp_->src_img2[(thread_yIndex + i) * imgWidth + (thread_xIndex + j)] -
            imp_->src_img2[((thread_yIndex + i) - 1) * imgWidth + (thread_xIndex + j)] -
            imp_->src_img2[(thread_yIndex + i) * imgWidth + (thread_xIndex + j) - 1] -
            imp_->src_img2[(thread_yIndex + i) * imgWidth + (thread_xIndex + j) + 1] -
            imp_->src_img2[((thread_yIndex + i) + 1) * imgWidth + (thread_xIndex + j)];
            imp_->lapla_img2[thread_index] = abs(gr);
            }
    }
     __syncthreads();

    int sad;

    if (xIndex > -1 * imp_->disparity && xIndex < imgWidth - 1 - imp_->blockSize && yIndex > 0 &&
        yIndex < imgHeight - 1 - imp_->blockSize)
    {
        sad = getSAD(imp_, xIndex, yIndex, imgHeight, imgWidth);

        if (sad < imp_->sadThreshold && sad > 0)
        {
            bool if_horizon = checkHorizontalInvariance(imp_, xIndex, yIndex, imgHeight, imgWidth);
            if (imp_->check_horizontal_invariance && if_horizon  == false)
            {
                int dj = xIndex + imp_->blockSize / 2;
                int di = yIndex + imp_->blockSize / 2;
                //bool if_true_disp = check_Disparity(imp_, dj, di, imgHeight, imgWidth);

                float true_disparity = imp_->depth_img[di * imgWidth + dj - 1] * imgWidth;
                float dis_error = true_disparity - (-1.0f * imp_->disparity);
            
                if (dis_error >= imp_->down_error) //&& dis_error <= up_error
                {
                    atomicAdd(&imp_->point_num, 1);

                   // __syncthreads();

                    imp_->camera_points[imp_->point_num * 3]     = dj;
                    imp_->camera_points[imp_->point_num * 3 + 1] = di;
                    imp_->camera_points[imp_->point_num * 3 + 2] = true_disparity;

                    imp_->pixel_points[imp_->point_num * 3    ] = xIndex;
                    imp_->pixel_points[imp_->point_num * 3 + 1] = yIndex;
                    imp_->pixel_points[imp_->point_num * 3 + 2] = sad;
                }

            }
        }
    }
}

__device__ int getSAD(image_process *imp_, int pxX, int pxY, int imgHeight, int imgWidth)
{
    int startX = pxX;
    int startY = pxY;
    int endX = pxX + imp_->blockSize - 1;
    int endY = pxY + imp_->blockSize - 1;

    int leftVal = 0, rightVal = 0;
    int sad = 0;

    for (int i = startY; i <= endY; i++)
    {
        // unsigned char *this_rowL = (unsigned char *)(imp_->src_img1 + (i * imgWidth));
        // unsigned char *this_rowR = (unsigned char *)(imp_->src_img2 + (i * imgWidth));
        // unsigned char *this_row_laplacianL = (unsigned char *)(imp_->lapla_img1 + (i * imgWidth));
        // unsigned char *this_row_laplacianR = (unsigned char *)(imp_->lapla_img2 + (i * imgWidth));
        for (int j = startX; j <= endX; j++)
        {
            unsigned char sL = imp_->lapla_img1[i * imgWidth + j];
            unsigned char sR = imp_->lapla_img2[i * imgWidth + j + imp_->disparity];

            leftVal += sL;
            rightVal += sR;

            unsigned char pxL = imp_->src_img1[i * imgWidth + j];
            unsigned char pxR = imp_->src_img2[i * imgWidth + j + imp_->disparity];

            sad += abs(pxL - pxR);
        }
    }
    int laplacian_value = leftVal + rightVal;
    if (leftVal < imp_->sobelLimit)
    {
        //ROS_INFO_STREAM("don't have enough texture:"<<leftVal);
        return -1;
    }
   // printf("laplacianl = %d, laplacianr = %d\n",leftVal,rightVal);
    //ROS_INFO_STREAM("have enough texture:"<<leftVal);

    if(laplacian_value != 0)
    {
        return 333 * (float)sad / (float)laplacian_value;
    }
    else
    {
        return -1;
    }
    
}

__device__ bool checkHorizontalInvariance(image_process *imp_, int pxX, int pxY, int imgHeight, int imgWidth)
{
    int startX = pxX;
    int startY = pxY;
    int endX = pxX + imp_->blockSize - 1;
    int endY = pxY + imp_->blockSize - 1;

    if (startX + imp_->zero_disparity + imp_->INVARIANCE_CHECK_HORZ_OFFSET_MIN < 0 || endX + imp_->zero_disparity + imp_->INVARIANCE_CHECK_HORZ_OFFSET_MAX > imgWidth)
    {
        return true;
    }
    if (startY + imp_->INVARIANCE_CHECK_VERT_OFFSET_MIN < 0 || endY + imp_->INVARIANCE_CHECK_VERT_OFFSET_MAX > imgHeight)
    {
        return true;
    }

    int leftVal = 0;
    int right_val_array[400];
    int sad_array[400];
    int sobel_array[400];

    for (int i = 0; i < 400; i++)
    {
        right_val_array[i] = 0;
        sad_array[i] = 0;
        sobel_array[i] = 0;
    }
    int counter = 0;

    for (int i = startY; i <= endY; i++)
    {
        for (int j = startX; j <= endX; j++)
        {
            unsigned char pxL = imp_->src_img1[i * imgWidth + j];
            unsigned char pxR_array[400], sR_array[400];
            counter = 0;
            for (int vert_offset = imp_->INVARIANCE_CHECK_VERT_OFFSET_MIN; vert_offset <= imp_->INVARIANCE_CHECK_VERT_OFFSET_MAX;
                 vert_offset += imp_->INVARIANCE_CHECK_VERT_OFFSET_INCREMENT)
            {
                for (int horz_offset = imp_->INVARIANCE_CHECK_HORZ_OFFSET_MIN; horz_offset <= imp_->INVARIANCE_CHECK_HORZ_OFFSET_MAX;
                     horz_offset++)
                {
                    pxR_array[counter] = imp_->src_img2[(i + vert_offset) *  imgWidth + j + imp_->zero_disparity + horz_offset];
                    sR_array[counter] = imp_->lapla_img2[(i + vert_offset) *  imgWidth + j + imp_->zero_disparity + horz_offset];
                    right_val_array[counter] += sR_array[counter];
                    sad_array[counter] += abs(pxL - pxR_array[counter]);
                    counter++;
                }
            }
            unsigned char sL = imp_->lapla_img2[i * imgWidth + j];
            leftVal += sL;
        }
    }

    int block_sad = 0;

    for (int i = 0; i < counter; i++)
    {
        sobel_array[i] = leftVal + right_val_array[i];
        
        if(sobel_array[i] != 0)
        {
            block_sad = 333 * 1 * (float)sad_array[i] / ((float)sobel_array[i]);
        }
        else
        {
            block_sad = 0;
        }
        
        if (right_val_array[i] >= imp_->sobelLimit &&  block_sad < imp_->sadThreshold)
        {                //counter = 8*8/2*2,每一个里面都是整个时视差窗口内12*12的总和，如果某一个counter内的值纹理丰富，则该点匹配，应当去掉
            return true; //是水平线，返回true
        }
    }
    return false;
}

__device__ bool check_Disparity(image_process *imp_, int pxX, int pxY, int imgHeight, int imgWidth)
{
    imp_->true_disparity = imp_->depth_img[pxY * imgWidth + pxX - 1] * imgWidth;
    float dis_error = imp_->true_disparity - (-1 * imp_->disparity);

    if (dis_error >= imp_->down_error) //&& dis_error <= up_error
    {
        return true;
    }
    else
    {
        return false;
    }
}

void image_process_gpu(image_process *imp_)
{
    //imp_->point_num = 0;
    // cudaEvent_t start, stop;
    // cudaEventCreate(&start);
    // cudaEventCreate(&stop);
    // cudaEventRecord(start, 0);

    // // 使用 std::chrono 来给算法计时
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    dim3 threadsPerBlock(16, 16);
    dim3 blocksPerGrid((imp_->imgWidth + threadsPerBlock.x - 1) / threadsPerBlock.x,
                       (imp_->imgHeight + threadsPerBlock.y - 1) / threadsPerBlock.y);
    image_process_cuda<<<blocksPerGrid, threadsPerBlock>>>(imp_, imp_->imgHeight, imp_->imgWidth);

    cudaError_t error = cudaGetLastError();
    printf("CUDA error: %s\n", cudaGetErrorString(error));

    // int rows = imp_->imgHeight;
    // int cols = imp_->imgWidth;

    // int stopi = rows;// - imp_->blockSize;
    // int startJ = -1 * imp_->disparity;
    // int stopJ = cols;// - imp_->blockSize;
    // if (imp_->disparity < 0)
    // {
    //      startJ = -imp_->disparity;
    //     stopJ = cols;// - imp_->blockSize;
    // }

    // int thread_num_i = stopi / imp_->blockSize + 1;
    // int thread_num_j = (stopJ - startJ) / imp_->blockSize + 1;
    // dim3 threadsPerBlock2(16, 16);
    // dim3 blocksPerGrid2((thread_num_j + threadsPerBlock.x - 1) / threadsPerBlock.x,
    //                     (thread_num_i + threadsPerBlock.y - 1) / threadsPerBlock.y);

    //image_process_sad_cuda<<<blocksPerGrid2, threadsPerBlock2>>>(imp_, imp_->imgHeight, imp_->imgWidth);
    
    cudaDeviceSynchronize();



    //cout<<"feature points num(cuda):"<<imp_->point_num<<endl;

    // cout<<"imgWidth,:"<<imp_->imgWidth<<endl;

    // cout<<"sadThreshold:"<<imp_->sadThreshold<<endl;
    // cout<<"sobelLimit:"<<imp_->sobelLimit<<endl;
    // cout<<"check_horizontal_invariance:"<<imp_->check_horizontal_invariance<<endl;

    // Mat dstImg(imp_->imgHeight, imp_->imgWidth , CV_8UC1, Scalar(0));

    // memcpy((unsigned char*)dstImg.data,(unsigned char*)imp_->lapla_img1, imp_->imgHeight * imp_->imgWidth * sizeof(unsigned char));
    // cv::namedWindow("laplacian_uma_cuda", 0);
    // cv::imshow("laplacian_uma_cuda", dstImg);
    // cv::waitKey(1);


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
    // ROS_INFO_STREAM("GPU:laplacian time:" << GPUTime_Global << "ms");
}
