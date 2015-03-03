
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <iostream>

//using namespace cv;

class Ransac {
public:
	Ransac();
	float slope = 1.5;
	float intersection = 250;
	double d;
	int allPointsLength;
	int runRansac(cv::Mat image);
	std::vector<CvPoint2D32f> allPoints;
	cv::Point linePoint1, linePoint2, testPoint, ransacPoint1, ransacPoint2;
private:
	const double maxDistance = 1.5;
	//const int minInliers = 200;
	int bestInliers = 0;

	void extractPoints(cv::Mat& img);
	void createLine();
	int count=0;
};
