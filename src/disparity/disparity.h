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
#include "../kernels/mykernels.h"

#include "../defines.h"

using namespace std;
//using namespace cv;


class Disparity {
public:
	cv::StereoSGBM sgbm;
	cv::gpu::DisparityBilateralFilter *dbf;  
	cv::gpu::StereoBM_GPU *gpubm; 
	cv::gpu::GpuMat imgLOIGPU, imgROIGPU, lowerDispFinished, upperDispFinished;

	cv::Mat imgL, imgR, dispLR, dispRL, dispRLLR;
	cv::Mat combinedDisps=cv::Mat::zeros(IMAGEHEIGHT,IMAGEWIDTH, CV_8UC1);
    cv::Mat temporalCombinedDisps=cv::Mat::zeros(IMAGEHEIGHT,IMAGEWIDTH, CV_8UC1);
    cv::Mat obstacleImage=cv::Mat::zeros(IMAGEHEIGHT,IMAGEWIDTH, CV_8UC1);
    cv::Mat postDarkImage=cv::Mat::zeros(IMAGEHEIGHT,IMAGEWIDTH, CV_8UC1);
    
	Disparity();
	void GPUBMdisparity();
	void temporalDisparity();
	void SGBMdisparity();
	void vDispThresholdedImage(cv::Mat dispImg, float slope, float intersection, float thresholdOffset);
	void removeDarkRegions(cv::Mat orgImgLoi);

	void generateVdisp(cv::Mat h_disp);
	void generateVdispCPU(cv::Mat dispImg) ;

	cv::Mat vdispU16 = cv::Mat::zeros(IMAGEHEIGHT, 255, CV_16U);
	cv::Mat vdisp = cv::Mat::zeros(IMAGEHEIGHT, 255, CV_8UC1);
	cv::Mat vdispCPU = cv::Mat::zeros(IMAGEHEIGHT, 255, CV_8UC1);

private:
	cv::Mat temporalDisp;
	cv::Vec3b pixelValueBGR;
    int pixelValueBlue;
    int pixelValueGreen;
    int pixelValueRed;
};
#endif