cmake_minimum_required(VERSION 2.8.3)
project(auto_flight)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 ")

# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0  -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
roscpp 
cv_bridge 
message_generation 
mavros 
mavros_msgs 
nav_msgs 
rospy 
std_msgs 
sensor_msgs 
image_transport 
tf 
camera_info_manager 
geometry_msgs 
octomap_msgs 
octomap_ros 
visualization_msgs 
geographic_msgs)

add_service_files(
  DIRECTORY srv
  FILES
destinate.srv
)

add_message_files(FILES
  auto_flight_octomap_state.msg
  auto_flight_state.msg
)
generate_messages(DEPENDENCIES
std_msgs
geometry_msgs
)

##thread
set(PTHREAD_LIB "")
find_package(Threads)
if(CMAKE_THREAD_LIBS_INIT)
  string(LENGTH ${CMAKE_THREAD_LIBS_INIT} _length)
  if(_length GREATER 2)
    string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 0 2 _prefix)
    if(${_prefix} STREQUAL "-l")
      math(EXPR _rest_len "${_length} - 2")
      string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 2 ${_rest_len} PTHREAD_LIB)
    endif()
  endif()
endif() 

#opencv
find_package(OpenCV  REQUIRED
#    NO_MODULE
#    PATHS /usr/local
#    NO_DEFAULT_PATH
    )
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories( ${OpenCV_LIBRARY_DIRS})
message(${OpenCV_INCLUDE_DIRS} )

#octomap
FIND_PACKAGE( octomap REQUIRED )
include_directories( ${OCTOMAP_INCLUDE_DIRS})

#Eigen
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen3_LIBRARIES ${EIGEN3_LIBRARIES})
message(${EIGEN3_INCLUDE_DIRS} )


##pcl
find_package( PCL REQUIRED COMPONENT common io filters )#COMPONENT common io 
include_directories( ${PCL_INCLUDE_DIRS} )
#add_definitions( ${PCL_DEFINITIONS} )
#link_directories( ${PCL_LIBRARY_DIRS})

#pcl vtk bug
get_directory_property(dir_defs DIRECTORY ${PROJECT_SOURCE_DIR} COMPILE_DEFINITIONS)
set(vtk_flags)
foreach(it ${dir_defs})
    if(it MATCHES "vtk*")
    list(APPEND vtk_flags ${it})
    endif()
endforeach()
foreach(d ${vtk_flags})
    remove_definitions(-D${d})
endforeach()


# ## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED)

# setup CUDA
find_package(CUDA)

# 指定CUDA头文件的搜索路径
include_directories(${CUDA_INCLUDE_DIRS})

# 使CMake支持C++11特性
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu++0x")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

# 使CUDA NVCC 编译器支持C++11特性
LIST(APPEND CUDA_NVCC_FLAGS -std=c++11;-O2)
LIST(APPEND CUDA_NVCC_FLAGS -Xcompiler;-fPIC)
#set(CUDA_NVCC_FLAGS -O3;-G;-g)

LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_35,code=sm_35)
LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_53,code=sm_53)
LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_61,code=sm_61)
LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_62,code=sm_62)

# setup openmp
FIND_PACKAGE( OpenMP REQUIRED)  
if(OPENMP_FOUND)  
message("OPENMP FOUND")  
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")  
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")  
endif()  

 catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp cv_bridge message_generation 
  mavros mavros_msgs nav_msgs
  rospy std_msgs sensor_msgs 
  image_transport tf 
  camera_info_manager 
  geometry_msgs
  octomap_msgs octomap_ros
)

#lib 
link_directories(${PROJECT_SOURCE_DIR}/lib)
set(AIRSIM_LIBS 
 ${EIGEN3_LIBRARIES}
 ${PTHREAD_LIB}
 mavlinkcom_catkin
 rpc_catkin
 airlib_catkin
)

set( THIRD_LIBS 
 ${OpenCV_LIBS} 
 ${PCL_LIBRARIES}
 ${OCTOMAP_LIBRARIES}
 -fopenmp
)

##include
include_directories( 
  include 
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/airsim
  ${PROJECT_SOURCE_DIR}/include/airsim/MavLinkCom
  ${PROJECT_SOURCE_DIR}/include/airsim/common_utils
  ${PROJECT_SOURCE_DIR}/include/airsim/AirLib
  ${PROJECT_SOURCE_DIR}/include/airsim/rpcinclude
  ${PROJECT_SOURCE_DIR}/include/airsim/droneshell
  ${EIGEN3_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS} 
  )

add_subdirectory( src )
add_subdirectory( test )