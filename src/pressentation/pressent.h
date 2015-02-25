
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <iostream>

//using namespace cv;

class Pressent {
public:
	Pressent();
	cv::Mat representGrayInColor(cv::Mat gray, int min, int max);
	
private:
	
};
