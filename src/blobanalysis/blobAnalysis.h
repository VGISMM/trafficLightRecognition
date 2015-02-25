#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

//using namespace cv;


class BlobAnalysis {
public:
	BlobAnalysis();
	cv::vector<cv::Rect> extractBlobs(cv::Mat);
	
private:
	cv::vector<cv::Rect> blobRects;
	cv::vector<cv::Rect> blobRectFound;

	cv::vector<cv::Point> blobCenterCurrent;
	cv::vector<cv::Point> blobCenterPrevious;
	cv::vector<cv::Point> blobCenterFound;
	float getDistance(cv::Point p1,cv::Point p2);
	int closestCenterPoint;
	float myDistance;

	int minBlobSize; 
	int maxBlobSize;
	cv::Mat frame;
	cv::Mat thresFrame;
	int threshold_value;
	int max_BINARY_value;
	int maxRadius;
};