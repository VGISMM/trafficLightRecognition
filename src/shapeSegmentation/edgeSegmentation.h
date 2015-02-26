/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>

#include "../defines.h"

using namespace std;

class EdgeSegmentation {
public:
  EdgeSegmentation();
  
  void findEdges(cv::Mat frame);
  
private:
 

};