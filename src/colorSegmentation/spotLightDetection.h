/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "spotLight/spotLight.h"
#include "../defines.h"

using namespace std;

class SpotLightDetection {
public:
  SpotLightDetection();
  void segmentSpotLights(cv::Mat intensityChannel, cv::Mat frame);
  cv::Mat topHat;
  cv::vector<SpotLight> detectedSpotLights;
  
  //cv::vector<cv::Rect> spotlightRects;
  //cv::vector<cv::circle> spotlightCircles;
private:
	cv::Mat floodMask=cv::Mat::zeros(IMAGEHEIGHT+2,IMAGEWIDTH+2, CV_8UC1);

};