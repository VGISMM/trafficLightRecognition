/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../../defines.h"

using namespace std;

class SpotLight {
public:
  SpotLight();
  cv::Rect ROI;
  float solidity;
  float floodRatio;
  float rectRatio;

private:
	
};