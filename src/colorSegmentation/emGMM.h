#include <stdio.h>
#include <iostream>
#include <time.h>

#include "opencv2/legacy/legacy.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


#include "../defines.h"
using namespace std;

class emGMM{
public:
	emGMM();
	
	void init();
	void predictFrame(cv::Mat frame);
	cv::Mat EmGmmAll;
	cv::Mat imgOut;
private:
	void totalNumOfTrainingSamplesRows();
	cv::Mat createTrainingSamplesYCbCr();
	int totalTraningSamplesRows;
	cv::Mat convertedYCrCb;
	cv::Mat myTrainingImage;
	cv::Mat trainingImage;
	CvEM em_model;
    CvEMParams params;
};
