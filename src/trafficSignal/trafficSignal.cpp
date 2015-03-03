#include "trafficSignal.h"

TrafficSignal::TrafficSignal(){
	

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
