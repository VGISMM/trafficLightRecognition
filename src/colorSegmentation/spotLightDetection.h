/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>

#include "../defines.h"

using namespace std;

class SpotLightDetection {
public:
  SpotLightDetection();
  void segmentSpotLights(cv::Mat frame);
  cv::Mat topHat;
private:


};