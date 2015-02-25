#include "Vehicle.h"
using namespace std;
Vehicle::Vehicle(){
	
	minDist=99.9;
	maxDist=0;
	avgDist=0;
}

void Vehicle::calcAvgDist() 
{
	for(int i=0;i<image2Dpositions.size();i++)
	{
		if (image2Dpositions[i].x < centerLeft)
		{
			leftCount++;
		}
		else if(image2Dpositions[i].x > centerLeft)
		{
			rightCount++;
		}
		else
		{
			centerCount++;
		}

		if (minDist > image2Dpositions[i].z)
		{
			minDist = image2Dpositions[i].z;
			minDistFoundAtFrame=foundAtFrame;
		}
		if (maxDist < image2Dpositions[i].z)
		{
			maxDist = image2Dpositions[i].z;
		}
		avgDist += image2Dpositions[i].z;
	}
	avgDist=avgDist/image2Dpositions.size();
}

void Vehicle::initKalman(cv::Point3f world3Dpoint) 
{	
	//cout << "world3Dpoint: " << world3Dpoint << endl; 
	vKalman.setKalman(world3Dpoint.x, world3Dpoint.y, world3Dpoint.z);
}

void Vehicle::predictVehicle()
{	
	vKalman.kalmanPredict();
}
 
void Vehicle::getVehiclePoint()
{	
	vehicleKalman3DPoint.x = vKalman.prediction.at<float>(0);
	vehicleKalman3DPoint.y = vKalman.prediction.at<float>(1);
	vehicleKalman3DPoint.z = vKalman.prediction.at<float>(2);

	//cout << "vehicleKalman3DPoint.x: " << vehicleKalman3DPoint.x << endl; 

	vehicleKalman2DPoint.x = (vKalman.estimated.at<float>(0)*focalLenth)/vKalman.estimated.at<float>(2);
    vehicleKalman2DPoint.y = (vKalman.estimated.at<float>(1)*focalLenth)/vKalman.estimated.at<float>(2);
    vehicleKalman2DPoint.z = (vKalman.estimated.at<float>(0)*focalLenth)/vehicleKalman2DPoint.x;

    //vehicleKalman2DPoints.push_back(cv::Point3f((vehicleKalman3DPoint.x*focalLenth)/vehicleKalman3DPoint.z, (vehicleKalman3DPoint.y*focalLenth)/vehicleKalman3DPoint.z, (vehicleKalman3DPoint.x*focalLenth)/vehicleKalman2DPoint.x));

}

void Vehicle::kalmanCorrect(cv::Point3f world3Dpoint) 
{	
	vKalman.Correct(world3Dpoint.x, world3Dpoint.y, world3Dpoint.z);
}

 