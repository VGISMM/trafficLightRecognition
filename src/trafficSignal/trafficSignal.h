 #ifndef _TRAFFICSIGNAL_
 #define _TRAFFICSIGNAL_
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

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

	std::vector<cv::Point3f> image2Dpositions;
	std::vector<cv::Point3f> world3Dpositions;
	std::vector<cv::Point3f> widthHeightDepth;
	float heightAboveRoad;
	//cv::vector<cv::Point3f> directionVectors;
	cv::Point3f vehicleKalman2DPoint;
	cv::Point3f vehicleKalman3DPoint;
	cv::Scalar trafficLightColor;

	int color;

	bool TP = false;
	bool TN = false;
	bool FP = true;

	//bool spotLight = false;
	//bool colorLight = false;
	float TLConfidence = 0;
	//float colorLightConfidence = 0;

	cv::Rect rect2d, lampRect;

	float rectRatio, solidity, floodRatio, colorConfidence, meanIntensity;
	//float spotLightRectRatio, spotLightSolidity, spotLightFloodRatio;
	//float colorConfidence, colorMeanIntensity, colorRectRatio, colorFloodRatio;

	cv::Point3f upperLeftCorner, lowerRightCorner, nearestPoint, rightPoint, leftPoint;
	
	int bestMatchIndex=0;
	
	int foundAtFrame;
	int lifeTime = 0;
	
	bool detectedBefore = false;
	void findTrafficSignalsImageCoordiantes();
	void calculateSpotLightConfidence();
	void calculateColorLightConfidence();

private:
	cv::Point3f projectFrom3Dto2D(cv::Point3f world3Dcoordinate);
};
 #endif