cmake_minimum_required(VERSION 2.8)
project(ADAS)
find_package(PCL 1.3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

ADD_EXECUTABLE(main 
            src/main.cpp
            src/disparity/disparity.cpp
            src/blobAnalysis/blobAnalysis.cpp
            src/colorSegmentation/backproject.cpp
            src/colorSegmentation/emGMM.cpp
            src/pressentation/pressentation.cpp
            src/pointCloud/pointCloud.cpp
            src/trafficSignal/trafficSignal.cpp
            )

TARGET_LINK_LIBRARIES(main ${PCL_LIBRARIES} ${OpenCV_LIBS})

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    # Other flags
)

