#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include "../defines.h"
#import "colorLight/colorLight.h"
#include <vector>

using namespace cv::ml;
using namespace std;

class GMM{
public:
	GMM();
	void init();
	void predictFrame();
	void detectColors();
  	std::vector<ColorLight> detectedColorLights;

	cv::Mat mything=cv::Mat::zeros(IMAGEHEIGHT/2,IMAGEWIDTH, CV_8UC3);
	//cv::Mat detectedColorRed, detectedColorGreen, detectedColorYellow;
	cv::Mat colorFrame, greenGMM, redGMM, yellowGMM, GMMoutput, converted, intensity;
	//int numberOfTrainingSamples = 20;
private:
	void locateTrafficLights(cv::Mat GMMChannel, int color);
	int findNumberOfTraningSamplePixels();
	cv::Mat combineTrainingSamples();

	cv::Mat myTrainingImage;
	cv::Mat trainingImage;
	cv::Ptr<EM> colorModel, gmmModelRed, gmmModelYellow, gmmModelGreen;
};
