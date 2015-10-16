#ifndef KALMAN_H
#define KALMAN_H

/*--------- INCLUDES------------*/
#include <stdio.h>
#include "cv.h"  
#include "highgui.h" 
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

//using namespace cv;
using namespace std;

/*--------- Class with attributes and methods declarations------------*/
class Kalman{
public:
	Kalman();
	void initKalman(float x, float y);
	void kalmanPredict();
	void kalmanCorrect(float x, float y);
	cv::Mat prediction, estimated;
private:
	cv::KalmanFilter KF;
	cv::Mat_<float> measurement = cv::Mat_<float>::zeros(2,1);
	//Mat_<float> measurement(2,1); 
	//Mat_<float> state(4, 1); // (x, y, Vx, Vy)

};
#endif