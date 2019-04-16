
cuda_add_library(image_cuda  SHARED 
cuda/edge_detector.cu
cuda/img_process_gpu.cu
)
target_link_libraries( image_cuda   ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})
add_dependencies(image_cuda ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

cuda_add_library(Flight SHARED
    Config.cpp
    Frame.cpp 
    ImgPro.cpp 
    StereoMap.cpp
    trajplanner/uavtrans.cpp
    trajplanner/csvparser.c
    trajplanner/searchdistance.cpp
    trajplanner/Trajectory.cpp
    trajplanner/TrajectoryLibrary.cpp
    )    
target_link_libraries( Flight ${THIRD_LIBS} ${AIRSIM_LIBS} ${catkin_LIBRARIES} image_cuda )
#add_dependencies(libFlight ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# cuda_add_library(img_process_gpu  SHARED 
# cuda/img_process_gpu.cu)
# target_link_libraries( img_process_gpu   
# ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   Flight
#   )
 
