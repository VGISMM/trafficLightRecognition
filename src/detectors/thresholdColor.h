#include <iostream>
#include <limits>
#include <vector>
#include <opencv2/opencv.hpp>

#include "../defines.h"
//#include "colorLight/colorLight.h"

#import "blobAnalysis.h"

using namespace cv::ml;
using namespace std;

class ThresholdColor{
public:
	ThresholdColor();
	void detectColors();
  	std::vector<ColorLight> detectedColorLights;

	cv::Mat mything=cv::Mat::zeros(IMAGEHEIGHT/2,IMAGEWIDTH, CV_8UC3);
	//cv::Mat detectedColorRed, detectedColorGreen, detectedColorYellow;
	cv::Mat colorFrame, allTC, allTCBlobs, greenTC, redTC, yellowTC, outputTC, converted, intensity;

	blobAnalysis colorThreshBlobAnalysis;
private:
	void locateTrafficLights(cv::Mat TCBlobs, cv::Mat TCChannel, int color);
		//cv::Mat thresholdFrameLUV(cv::Mat frame);
	void thresholdFrameHSV();
};

  

  