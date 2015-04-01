#include "trafficSignal.h"

TrafficSignal::TrafficSignal(){}

void TrafficSignal::calculateSpotLightConfidence()
{
	// spotLightConfidence is a measure of the various ratio's(make sure that each one is < 1),
	// we vant spotLightConfidence to be high when the ratios are good, to reward having several detectors find the same TL. 
	spotLightConfidence = 1/(1+((abs(spotLightRectRatio)/3)+(abs(spotLightSolidity)/3)+(abs(spotLightFloodRatio)/3)));
}

void TrafficSignal::calculateColorLightConfidence()
{
    //cout << colorFloodRatio << endl;
	colorLightConfidence = 1/(1+((abs(colorConfidence)/3)+(abs(colorRectRatio)/3)+(abs(colorFloodRatio)/3)));
}

void TrafficSignal::findTrafficSignalsImageCoordiantes()
{
	trafficSignalPosition2D = projectFrom3Dto2D(trafficSignalPosition);
    trafficSignalPlanePoint2D = projectFrom3Dto2D(trafficSignalPlanePoint);
}

cv::Point3f TrafficSignal::projectFrom3Dto2D(cv::Point3f world3Dcoordinate)
{
    cv::Point3f world2Dcoordiante;
    world2Dcoordiante.x = (world3Dcoordinate.x*FOCALLENTH)/world3Dcoordinate.z;
    world2Dcoordiante.y = (world3Dcoordinate.y*FOCALLENTH)/world3Dcoordinate.z;
    world2Dcoordiante.z = (world3Dcoordinate.x*FOCALLENTH)/world2Dcoordiante.x;
    return world2Dcoordiante;
}
