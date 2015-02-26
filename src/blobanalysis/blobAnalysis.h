#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <iostream>
#include <vector_types.h>
#include <string>
#include <stdio.h>
//using namespace cv;
using namespace std;

class BlobAnalysis {
public:
	BlobAnalysis();
	void extractBlobs(cv::Mat);
	cv::vector<cv::Rect> blobRects;
private:
	float minBlobSize, maxBlobSize; 
	cv::Mat frame;
};