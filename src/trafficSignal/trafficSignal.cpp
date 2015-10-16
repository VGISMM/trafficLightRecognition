#include "trafficSignal.h"

TrafficSignal::TrafficSignal(){}

void TrafficSignal::calculateSpotLightConfidence()
{
    int components = 5;
	// spotLightConfidence is a measure of the various ratio's(make sure that each one is < 1),
	// we vant spotLightConfidence to be high when the ratios are good, to reward having several detectors find the same TL. 
	TLConfidence = 1/(1+((abs(colorConfidence)/components)+(abs(rectRatio)/components)+(abs(solidity)/components)+(abs(floodRatio)/components)+(abs(meanIntensity)/components)));
}

void TrafficSignal::calculateColorLightConfidence()
{
    //cout << colorConfidence << " " << rectRatio << " " << solidity << " " << floodRatio << " " << meanIntensity << endl;
	TLConfidence = colorConfidence + rectRatio + solidity + floodRatio + meanIntensity;
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
