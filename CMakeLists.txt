cmake_minimum_required(VERSION 2.8)
project(gpu_testing)

find_package(OpenCV REQUIRED)

find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS "-arch=sm_30" CACHE STRING "nvcc flags" FORCE)
set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)

include_directories(${CUDA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
cuda_add_executable(main 
            src/main.cpp
            src/disparity/disparity.cpp
            src/blobAnalysis/blobAnalysis.cpp
            src/colorSegmentation/backproject.cpp
            src/kernels/mykernel.cu)
target_link_libraries(main ${CUDA_LIBRARIES} ${OpenCV_LIBS})

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    # Other flags
)