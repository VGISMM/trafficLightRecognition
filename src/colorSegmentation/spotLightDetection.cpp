#include "spotLightDetection.h"

SpotLightDetection::SpotLightDetection(){} 

void SpotLightDetection::segmentSpotLights(cv::Mat intensityChannel, cv::Mat frame){
  cv::Mat compareMat = frame.clone();
  //int morph_size = 1;
  //cv::Mat element = getStructuringElement( cv::MORPH_RECT, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
  cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
  cv::Mat sel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
  //morphologyEx(intensityChannel, topHat, cv::MORPH_TOPHAT, sel, cv::Point(-1,-1), 9 );
  morphologyEx(intensityChannel, topHat, cv::MORPH_TOPHAT, sel1, cv::Point(-1,-1), 4 );
  erode(topHat, topHat, sel, cv::Point(-1, -1), 1, 1, 1);
  morphologyEx( topHat, topHat, cv::MORPH_OPEN, sel, cv::Point(-1,-1), 1 );
  //imshow("topHatORg",topHat);
  
  cv::vector<cv::vector<cv::Point>> contours; 
  cv::vector<cv::Vec4i> hierarchy;

  threshold( topHat, topHat, 40, 255,0 );
  //imshow("topHatORg1",topHat);
  findContours(topHat.clone(), contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  cv::Mat out;
  cv::Rect tempRect, floodRect;
  cv::vector<cv::vector<cv::Point>> hullPoints(contours.size());

  for( int i = 0; i< contours.size(); i++ )
  { 
    tempRect = boundingRect(contours[i]);
    
    float rectRatio;
    if(tempRect.width < tempRect.height)
    {
      rectRatio = (float)tempRect.width/(float)tempRect.height;
    }
    else
    {
      rectRatio = (float)tempRect.height/(float)tempRect.width;
    }
    if (tempRect.width > minBoundingRectDim && tempRect.width < maxBoundingRectDim && tempRect.height > minBoundingRectDim && tempRect.height < maxBoundingRectDim && rectRatio > minBoundingRectRatio)
    {
      float tempRectArea = (float)tempRect.width*tempRect.height;
      convexHull(contours[i],hullPoints[i],false);
      float convexArea = contourArea(hullPoints[i]);
      float solidity;
      if(convexArea < tempRectArea)
      {
        solidity = convexArea/tempRectArea;
      }
      else
      {
        solidity = tempRectArea/convexArea;
      }
      rectangle(compareMat, tempRect, cv::Scalar( 0, 255, 0 ), 1, 4 );
      if (solidity > 0.4)
      {  
        cv::Mat tempMat = intensityChannel(tempRect);

        /*
        // Using id of median intensity pixel as seed, center column
        cv::Mat sortedIndexes;
        cv::sortIdx(tempMat, sortedIndexes, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
        int medianRowIndex = sortedIndexes.at<uchar>(sortedIndexes.rows/2, sortedIndexes.cols/2);
        floodFill(intensityChannel.clone(), cv::Point(tempRect.x+tempRect.width/2, tempRect.y+medianRowIndex), cvScalar(255,0,0), &floodRect1, cvScalarAll(20), cvScalarAll(30), 8 | ( 255 << 8 ) | cv::FLOODFILL_FIXED_RANGE);
        */
        // alternative using id of max intensity pixel as seed
        double max;
        cv::Point maxPoint;
        minMaxLoc(tempMat, NULL, &max, NULL, &maxPoint);
        floodFill(intensityChannel.clone(), cv::Point(tempRect.x+maxPoint.x, tempRect.y+maxPoint.y), cvScalar(255,0,0), &floodRect, cvScalarAll(50), cvScalarAll(20), 8 | ( 255 << 8 ) | cv::FLOODFILL_FIXED_RANGE);
        float floodArea = (float)floodRect.width*floodRect.height;
        float floodRatio;
        if(floodArea < tempRectArea)
        {
          floodRatio = floodArea/tempRectArea;
        }
        else
        {
          floodRatio = tempRectArea/floodArea;
        }
        if (floodRatio > 0.30)
        {

          //rectangle(compareMat, floodRect, cv::Scalar( 255, 255, 0 ), 1, 4 );
          //cout << "Contour index: " << i << " solidity: " << solidity << " tempRectArea: " << tempRectArea << " floodArea: " << floodArea << " floodRatio: " << floodRatio << " rectRatio: " << rectRatio << endl;
          SpotLight tempDetectedSpotLight;
          tempDetectedSpotLight.ROI = tempRect;
          tempDetectedSpotLight.rectRatio = rectRatio;
          tempDetectedSpotLight.floodRatio = floodRatio;
          tempDetectedSpotLight.solidity = solidity;
          detectedSpotLights.push_back(tempDetectedSpotLight); 
        }
      }
    }
  
  //for( int i = 0; i< detectedSpotLights.size(); i++ )
  //{
  //  rectangle(compareMat, detectedSpotLights[i].ROI, cv::Scalar( 0, 0, 255 ), 1, 4 );
  //}
  //cv::resize(compareMat,compareMat,cv::Size(),0.5,0.5,CV_INTER_LINEAR);
  //imshow("spotlights",compareMat);   
  }

/*
  cv::Mat floodMask=cv::Mat::zeros(IMAGEHEIGHT+2,IMAGEWIDTH+2, CV_8UC1);
  float circleRadius;
  for( int i = 0; i< contours.size(); i++ )
  { 
    minEnclosingCircle(contours[i], circleCenters, circleRadius);
    
    if (circleRadius > 1 && circleRadius < 15)
    {
      float circleArea = M_PI*(circleRadius*circleRadius);
      convexHull(contours[i],hullPoints[i],false);
      float solidity = float(circleArea)/contourArea(hullPoints[i]);
      if (solidity < 2)
      {  
        
        // floodfill squares:  
        floodFill(frame.clone(), circleCenters, cvScalar(255,0,0), &floodRect, cvScalarAll(50), cvScalarAll(15), cv::FLOODFILL_FIXED_RANGE);
        float sizeRatio = ((float)floodRect.width*floodRect.height)/circleArea;

        if (sizeRatio < 1.5 && sizeRatio > 0.25 && floodRect.width < 30 && floodRect.height < 30)
        {
          cout << "index: " << i << " solidity: " << solidity << " sizeRatio: " << sizeRatio << endl;
          //rectangle(frame, tempRect, cv::Scalar( 0, 255, 0 ), 2, 3 );
          rectangle(frame, floodRect, cv::Scalar( 255, 255, 0 ), 2, 4 );
          circle(frame, circleCenters, circleRadius, cv::Scalar(255,0,50), 2, 8, 0);
        }
        

        // floodfill circles:
        floodMask.setTo(cv::Scalar(0));
        floodFill(frame.clone(), floodMask, circleCenters, cvScalar(255,0,0), 0, cvScalarAll(80), cvScalarAll(25), cv::FLOODFILL_FIXED_RANGE);
        cv::vector<cv::vector<cv::Point>> floodContours;
        cv::vector<cv::Vec4i> floodHierarchy;
        
        
        findContours(floodMask, floodContours, floodHierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        cv::Point2f floodCircleCenters;
        float floodCircleRadius;
        minEnclosingCircle(floodContours[0], floodCircleCenters, floodCircleRadius);
        float floodCircleArea = M_PI*(floodCircleRadius*floodCircleRadius);
        float sizeRatio = (float)floodCircleArea/circleArea;

        if (sizeRatio < 12.5 && sizeRatio > 0.15)
        {
          cout << "index: " << i << " solidity: " << solidity << " sizeRatio: " << sizeRatio << endl;
          //rectangle(frame, tempRect, cv::Scalar( 0, 255, 0 ), 2, 3 );
          rectangle(frame, floodRect, cv::Scalar( 255, 255, 0 ), 2, 4 );
          circle(frame, circleCenters, circleRadius, cv::Scalar(255,0,50), 2, 8, 0);
          circle(frame, floodCircleCenters, floodCircleRadius, cv::Scalar(0,255,50), 2, 8, 0);
          //imshow("floodMask",floodMask);
          //cv::waitKey();
        }

      }
    }    
  }
*/
  /*for( int i = 0; i< spotlightRects.size(); i++ )
    {
    //rectangle(frame, spotlightRects[i], cv::Scalar( 0, 55, 255 ), 2, 4 );
    }*/
    //cv::resize(frame,frame,cv::Size(),0.5,0.5,CV_INTER_LINEAR);
    //cv::resize(compareMat,compareMat,cv::Size(),0.5,0.5,CV_INTER_LINEAR);
    //imshow("compareMat",compareMat);
    //imshow("frame",frame);

    
  //threshold( topHat, topHat, 30, 255,0 );
  
  //cv::Mat cloneTop = topHat.clone();
  //cv::resize(cloneTop,cloneTop,cv::Size(),0.5,0.5,CV_INTER_LINEAR);
  //imshow("topHat",cloneTop);
}
