/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "../defines.h"
//#include "colorLight/colorLight.h"

#import "blobAnalysis.h"

using namespace std;

class Backproject {
public:
  Backproject();
  
  void init();
  void detectColors();
  std::vector<ColorLight> detectedColorLights;
  cv::Mat mything=cv::Mat::zeros(IMAGEHEIGHT/2,IMAGEWIDTH, CV_8UC3);
  cv::Mat colorFrame, LUVFrame, greenBP, redBP, yellowBP, outBP, intensity;

  blobAnalysis backprojBlobAnalysis;
private:
  
  cv::Mat greenHistogram, redHistogram, yellowHistogram, objectHistogram1, objectHistogram2, globalHistogram;
  void locateTrafficLights(cv::Mat BPChannel, cv::Mat blobChannel, int color);
  cv::Mat showHistogram(cv::Mat histogram);
  

};