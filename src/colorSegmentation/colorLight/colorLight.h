/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../../defines.h"

using namespace std;

class ColorLight {
public:
  ColorLight();
  cv::Rect ROI;
  int color;
  float confidence, rectRatio, floodRatio;

private:
	
};