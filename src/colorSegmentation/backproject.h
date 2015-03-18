/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>

#include "../defines.h"

using namespace std;

class Backproject {
public:
  Backproject();
  
  void init();
  void backproject(cv::Mat frame);

  cv::Mat greenBP, redBP, outBP;
private:
  
  cv::Mat greenHistogram, redHistogram, objectHistogram1, objectHistogram2, globalHistogram;
  cv::Mat showHistogram(cv::Mat);


};