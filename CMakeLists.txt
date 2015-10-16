cmake_minimum_required(VERSION 2.8)
project(ADAS)

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

find_package(OpenCV REQUIRED)
#find_package(CUDA REQUIRED)
#set(CUDA_NVCC_FLAGS "-arch=sm_30" CACHE STRING "nvcc flags" FORCE)
#set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
#include_directories(${CUDA_INCLUDE_DIRS})

#SET(LIB_TYPE STATIC)    #set the lib type
#CUDA_ADD_LIBRARY(myCUDA ${LIB_TYPE} src/stereo/kernels/vdispkernel.h src/stereo/kernels/vdispkernel.cu)

#find_package(PCL 1.3 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

include_directories( ${OpenCV_INCLUDE_DIRS})
ADD_EXECUTABLE(main 
            src/main.cpp
            #src/stereo/disparity/disparity.cpp
            src/detectors/blobAnalysis.cpp
            src/candidateEvaluator/candidateEvaluator.cpp
            src/evaluation/evaluation.cpp
            src/detectors/backproject.cpp
            src/detectors/colorLight/colorLight.cpp
            #src/detectors/GMM.cpp
            src/detectors/spotLightDetection.cpp
            src/detectors/spotLight/spotLight.cpp
            src/detectors/thresholdColor.cpp
            src/detectors/edgeSegmentation.cpp
            src/pressentation/pressentation.cpp
            #src/stereo/pointCloud/pointCloud.cpp
            src/trafficSignal/trafficSignal.cpp
            src/stereo/ransac/ransac.cpp
            src/tracking/kalmanfilter2d.cpp
            #src/stereo/kernels/mykernels.cpp
            )
            #${PCL_LIBRARIES} myCUDA ${CUDA_LIBRARIES}
target_link_libraries(main ${OpenCV_LIBS})

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    # Other flags
)
