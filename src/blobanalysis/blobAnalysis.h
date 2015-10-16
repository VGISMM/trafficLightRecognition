#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>
#include <stdio.h>

#include "../defines.h"
//using namespace cv;
using namespace std;

class BlobAnalysis {
public:
	BlobAnalysis();
	void extractBlobs(cv::Mat, cv::Mat);
	std::vector<cv::Rect> blobRects;
	std::vector<cv::Rect> biggerBlobRects;
private:
	float minBlobSize, maxBlobSize; 
	cv::Mat frame;
};