 #ifndef _TRAFFICSIGNAL_
 #define _TRAFFICSIGNAL_
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <stdio.h>
#include <iostream>

#include "../defines.h"
#include "../tracking/kalmanfilter2d.h"
using namespace std;
//using namespace cv;

class TrafficSignal {
public:
	TrafficSignal();
	cv::Point3f trafficSignalPosition, trafficSignalPosition2D, trafficSignalPlanePoint, trafficSignalPlanePoint2D;

	cv::vector<cv::Point3f> image2Dpositions;
	cv::vector<cv::Point3f> world3Dpositions;
	cv::vector<cv::Point3f> widthHeightDepth;
	float heightAboveRoad;
	//cv::vector<cv::Point3f> directionVectors;
	cv::Point3f vehicleKalman2DPoint;
	cv::Point3f vehicleKalman3DPoint;

	cv::Rect rect2d;
	cv::Point3f upperLeftCorner, lowerRightCorner, nearestPoint, rightPoint, leftPoint;
	
	int bestMatchIndex=0;
	
	int foundAtFrame;
	int lifeTime = 0;
	
	bool detectedBefore = false;
	void findTrafficSignalsImageCoordiantes();

private:
	cv::Point3f projectFrom3Dto2D(cv::Point3f world3Dcoordinate);
};
 #endif