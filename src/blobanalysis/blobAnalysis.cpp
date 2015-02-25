#include "BlobAnalysis.h"


/************************************
 * Function for initializing values *
 ************************************/
BlobAnalysis::BlobAnalysis	() {
	minBlobSize=100; 
	maxBlobSize=700000;
	threshold_value=8;
	max_BINARY_value=255;
	maxRadius=1050;
}

cv::vector<cv::Rect> BlobAnalysis::extractBlobs(cv::Mat frame){ 
	cv::vector<cv::vector<cv::Point>> contours; 
	cv::vector<cv::Vec4i> hierarchy;
	cv::Rect bounding_rect;
	thresFrame=cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);;
	blobRects.clear();

	blobCenterFound.clear();
	blobCenterCurrent.clear();
	blobRectFound.clear();

	threshold( frame, thresFrame, threshold_value, max_BINARY_value,cv::THRESH_BINARY);
	
	cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
	cv::Mat sel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
	//dilate(thresFrame, thresFrame, Mat(), Point(-1, -1), 6, 1, 3);
	erode(thresFrame, thresFrame, cv::Mat(), cv::Point(-1, -1), 3, 1, 5);
	morphologyEx( thresFrame, thresFrame, cv::MORPH_OPEN, sel, cv::Point(-1,-1), 1 );
	dilate(thresFrame, thresFrame, sel1, cv::Point(-1, -1), 9, 1, 3);
	//erode(thresFrame, thresFrame, Mat(), Point(-1, -1), 5, 1, 5);

    
    //imshow("Thresholded",thresFrame);

    /** Find the contours in the image **/
	findContours( thresFrame.clone(), contours, hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE  ); 

	/** iterate through each contour **/
	for( int i = 0; i< contours.size(); i++ ){

		/** Find the area of contour **/
		double a=contourArea( contours[i],false);

		/** Make sure the blob is within a defined size **/
		if( (a<maxBlobSize) && (a>minBlobSize) ) { 

			/** Find the bounding rectangle for biggest contour **/
			bounding_rect=boundingRect(contours[i]); 

			/** Create vector of rect containing all found blobs **/
			blobRects.push_back(bounding_rect);  

			/** Calc center point to determine movement **/
			cv::Point center=cv::Point(bounding_rect.x+bounding_rect.width/2,bounding_rect.y+bounding_rect.height/2); 

			/** Push all center points to vector **/
			blobCenterCurrent.push_back(center); 
		}
    }

    /** Run through all blobCenters in current frame **/
    for (int n = 0; n < blobCenterCurrent.size(); ++n){ 
    	int minDistance=999999;

    	/** Run through all blobCenters in previous frame **/
    	for (int j = 0; j < blobCenterPrevious.size(); ++j) // 
    	{
    		/** Calc euclidean distance between two points **/
    		myDistance=getDistance(blobCenterCurrent[n],blobCenterPrevious[j]);

    		/** Find the closest point between current frame and previous frame **/
    		if (( myDistance < minDistance)) // 
				{
					minDistance=myDistance;
					closestCenterPoint=n;
				}
    	}

    	/** Determine if the closest point is "close enough" **/
    	if (minDistance<maxRadius) // 
    	{
    		/** Push the Rect information to a vector (Note: not center point) **/
    		blobRectFound.push_back(blobRects[closestCenterPoint]);

    	}
    }
    blobCenterPrevious.clear();
    /** Save current blobs for use in next iteration. **/
    blobCenterPrevious=blobCenterCurrent; // 

	return blobRectFound;
}

/** Function for calculating the euclidian distance between two points **/
float BlobAnalysis::getDistance(cv::Point p1,cv::Point p2) {
	return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
}