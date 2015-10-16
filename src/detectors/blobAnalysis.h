/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>

#include "colorLight/colorLight.h"
#include "../defines.h"

using namespace std;

class blobAnalysis {
public:
  blobAnalysis();
  void calculateScores(cv::Mat blobChannel, cv::Mat intensityChannel, cv::Mat colorFrame);

  std::vector<ColorLight> detectedTLs;
  cv::Mat presentationImage;
  std::vector<std::vector<cv::Point>> contours; 
  std::vector<cv::Vec4i> hierarchy;
  //cv::vector<cv::Rect> spotlightRects;
  //cv::vector<cv::circle> spotlightCircles;
private:
	cv::Mat floodMask=cv::Mat::zeros(IMAGEHEIGHT/2+2,IMAGEWIDTH+2, CV_8UC1);

};