/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "colorLight/colorLight.h"
#include "../defines.h"
#include "blobAnalysis.h"

using namespace std;

class SpotLightDetection {
public:
  SpotLightDetection();
  void segmentSpotLights();
  cv::Mat topHat, topHatBlobs;
  cv::Mat intensityChannel;
  std::vector<ColorLight> detectedTLs;
  
  blobAnalysis spotLightBlobAnalysis;

  cv::Mat colorFrame, presentationImage;

  //cv::vector<cv::Rect> spotlightRects;
  //cv::vector<cv::circle> spotlightCircles;
private:
	cv::Mat floodMask=cv::Mat::zeros(IMAGEHEIGHT/2+2,IMAGEWIDTH+2, CV_8UC1);

};