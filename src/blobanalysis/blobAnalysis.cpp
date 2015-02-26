#include "blobAnalysis.h"

/************************************
 * Function for initializing values *
 ************************************/
BlobAnalysis::BlobAnalysis	() {
	minBlobSize=10; 
	maxBlobSize=100;
}

void BlobAnalysis::extractBlobs(cv::Mat frame){ 
	cv::vector<cv::vector<cv::Point>> contours; 
	cv::vector<cv::Vec4i> hierarchy;

	cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
	cv::Mat sel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
	dilate(frame, frame, sel, cv::Point(-1, -1), 1, 1, 3);
	morphologyEx( frame, frame, cv::MORPH_CLOSE, sel1, cv::Point(-1,-1), 1 );
	morphologyEx( frame, frame, cv::MORPH_OPEN, sel, cv::Point(-1,-1), 1 );
	
	//erode(frame, frame, Mat(), Point(-1, -1), 5, 1, 5);

    //imshow("Thresholded",frame);
	findContours( frame.clone(), contours, hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE  ); 
	//cv::Rect tempRect;
	for( int i = 0; i< contours.size(); i++ )
	{	
		// Find the area of contour
		double area=contourArea( contours[i],false);
		//cout << "size(): " << a << endl;
		if( (area<maxBlobSize) && (area>minBlobSize) ) { 
			blobRects.push_back(boundingRect(contours[i])); 
		} 
    }
}
