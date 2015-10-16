
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <iostream>

//using namespace cv;
using namespace std;
class Pressentation {
public:
	Pressentation();
	cv::Mat representGrayInColor(cv::Mat gray, int min, int max);
	
private:
	
};
