#include "blobAnalysis.h"

/************************************
 * Function for initializing values *
 ************************************/
BlobAnalysis::BlobAnalysis	() {
	minBlobSize=10; 
	maxBlobSize=100;
}

void BlobAnalysis::extractBlobs(cv::Mat frame, cv::Mat dstCbUV){ 
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

	cv::Mat clone, detected_edges;
	clone = dstCbUV.clone();
    //Canny(dstCbUV, detected_edges, 190, 255, 3);
    //cv::Mat test = cv::Mat::zeros(460+2,1280+2,CV_8UC1);
    //detected_edges.copyTo(test(cv::Rect(0,0,1280,460)));
	cv::Rect tempRect, rect;
	
	for( int i = 0; i< contours.size(); i++ )
	{	
		tempRect = boundingRect(contours[i]);
		//floodFill(clone, cv::Point(tempRect.x+tempRect.width/2,tempRect.y+tempRect.height/2), cvScalar(255,0,0), & rect, cvScalarAll(53.5), cvScalarAll(23.5), cv::FLOODFILL_MASK_ONLY);
		floodFill(clone, cv::Point(tempRect.x+tempRect.width/2,tempRect.y+tempRect.height/2), cvScalar(255,0,0), & rect, cvScalarAll(50), cvScalarAll(10), cv::FLOODFILL_FIXED_RANGE);
		// Find the area of contour
		if (rect.width > 3 && rect.width < 30 && rect.height > 3 && rect.height < 30)
		{
			blobRects.push_back(rect); 
		}
		
		//cv::waitKey();
    }

    bool intersect = false;
    bool winner = false;
    // check for intersecting rects
    for( int i = 0; i< blobRects.size(); i++ )
	{	
		cv::Rect r1 = cv::Rect(blobRects[i].x, blobRects[i].y, blobRects[i].width+40, blobRects[i].height+40);
		for( int j = 0; j< blobRects.size(); j++ )
		{
			if (i != j)
			{	
				cv::Rect r2 = cv::Rect(blobRects[j].x, blobRects[j].y, blobRects[j].width+40, blobRects[j].height+40);
				
				cv::Rect intersectRect = r1 & r2; // 
				if (intersectRect == cv::Rect())
				{
					intersect = true;
					if((r1.width*r1.height)>=(r2.width*r2.height))
					{
						winner = true;
					}
					else
					{
						winner = false;
					}
				}
				else
				{
					intersect = false;
				}
				
			}
    	}
		if (intersect)
		{
			if (winner)
			{
				biggerBlobRects.push_back(r1);
			}
		}
		else
		{
			biggerBlobRects.push_back(r1);
		}
    }
    //imshow("clone",clone);
    /*
	for( int i = 0; i< contours.size(); i++ )
	{	
		double area=contourArea( contours[i],false);
		//cout << "size(): " << a << endl;
		if( (area<maxBlobSize) && (area>minBlobSize) ) { 
			blobRects.push_back(tempRect); 
		} 
	} */
}
