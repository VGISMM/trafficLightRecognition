/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>

#include "../defines.h"

class EdgeSegmentation {
public:
  EdgeSegmentation();
  cv::Mat edgeMap;
  void findEdges(cv::Mat frame);
  std::vector<cv::Vec3f> circles;
private:
 

};