#ifndef DISPARITY_H
#define DISPARITY_H
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/gpu/gpumat.hpp"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <time.h>
#include <thread>

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <cuda_runtime.h>

#include "../defines.h"
using namespace std;
//using namespace cv;
void your_rgba_to_greyscale(unsigned char* const d_image,
                            unsigned int* const d_vdispImage, 
                            size_t numRows, size_t numCols, size_t vdispNumRows, size_t vdispNumCols);


class Disparity {
public:

	cv::gpu::DisparityBilateralFilter *dbf;  
	cv::gpu::StereoBM_GPU *gpubm; 
	cv::gpu::GpuMat imgLOIGPU, imgROIGPU, imgLOIGPUflip, imgROIGPUflip;
	cv::gpu::GpuMat dispLRGPU, dispRLGPU, dispRLGPUflip, dispRLLRGPU, dispFinished;
	cv::Mat imgL, imgR, dispLR, dispRL, dispRLLR;
	cv::Mat combinedDisps=cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
    cv::Mat temporalCombinedDisps=cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
    cv::Mat obstacleImage=cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
    cv::Mat postDarkImage=cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
    
	Disparity();
	void disparityImages();
	void temporalDisparity();
	void vDispThresholdedImage(float slope, float intersection, float thresholdOffset);
	void removeDarkRegions(cv::Mat orgImgLoi);

	//void reduceNumberOfBins(cv::Mat dispImg);
	void generateVdispAlt();
	void generateVdisp(cv::Mat h_disp);
	void generateUdisp(cv::Mat dispImg);
	//void generateVdispPlanes(cv::Mat dispImg);
	//void generateUdispPlanes(cv::Mat dispImg);

	cv::Mat vdispU16 = cv::Mat::zeros(imageHeight, 255, CV_16U);
	cv::Mat vdisp = cv::Mat::zeros(imageHeight, 255, CV_8UC1);
	cv::Mat Udisp = cv::Mat::zeros(255, imageWidth, CV_8UC1);
	//cv::Mat reducedImg = cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
	cv::Mat VdispPlanes = cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
	cv::Mat UdispPlanes = cv::Mat::zeros(imageHeight,imageWidth, CV_8UC1);
private:
	cv::Mat temporalDisp;
	cv::Vec3b pixelValueBGR;
    int pixelValueBlue;
    int pixelValueGreen;
    int pixelValueRed;
};
#endif