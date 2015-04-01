/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>

#include "../defines.h"
#include "colorLight/colorLight.h"

using namespace std;

class Backproject {
public:
  Backproject();
  
  void init();
  void backproject();
  cv::vector<ColorLight> detectedColorLights;

  cv::Mat colorFrame, greenBP, redBP, outBP, intensity;
private:
  
  cv::Mat greenHistogram, redHistogram, objectHistogram1, objectHistogram2, globalHistogram;
  void locateTrafficLights(cv::Mat BPChannel, int color);
  cv::Mat showHistogram(cv::Mat histogram);
  

};