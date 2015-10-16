#include "blobAnalysis.h"

blobAnalysis::blobAnalysis(){} 

void blobAnalysis::calculateScores(cv::Mat blobChannel, cv::Mat intensityChannel, cv::Mat colorFrame)
{
  findContours(blobChannel.clone(), contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  cv::Mat out;
  cv::Rect tempRect, floodRect;
  std::vector<std::vector<cv::Point>> hullPoints(contours.size());

  cvtColor(blobChannel, presentationImage, CV_GRAY2BGR);

  for( int i = 0; i< contours.size(); i++ )
  { 
    tempRect = boundingRect(contours[i]);

    float BBRatio;
    if(tempRect.width < tempRect.height)
    {
      BBRatio = (float)tempRect.width/(float)tempRect.height;
    }
    else
    {
      BBRatio = (float)tempRect.height/(float)tempRect.width;
    }
    if (tempRect.width > minBoundingRectDim && tempRect.width < maxBoundingRectDim && tempRect.height > minBoundingRectDim && tempRect.height < maxBoundingRectDim && BBRatio > minBoundingRectRatio)
    {
      float tempRectArea = (float)tempRect.width*(float)tempRect.height;

      // Solidity is the ratio between the area the convex blob and a perfect circle with approximately the same radius. Good for solid detections, not so much for halo detections
      float aproxRadius = ((float)tempRect.width+(float)tempRect.height)/4;
      float tempCircleArea = M_PI*aproxRadius*aproxRadius;      
      convexHull(contours[i], hullPoints[i], false);
      float convexArea = contourArea(hullPoints[i]);
      float solidity;
      if(convexArea < tempCircleArea)
      {
        solidity = convexArea/tempCircleArea;
      }
      else
      {
        solidity = tempCircleArea/convexArea;
      }

      // Find mean intensity of intensityChannel, corresponding to a type of confidence from the detector
      cv::Mat tempMat = intensityChannel(tempRect);
      cv::Scalar meanIntensity = mean(tempMat);

      //Find confidence in dominant color inside BB
      cv::Mat colorCrop = colorFrame(tempRect);
      cvtColor(colorCrop, colorCrop, CV_BGR2HSV);
      int finalColor;
      float colorConfidence;
      int voteRed = 0; //0
      int voteYellow = 0; //1
      int voteGreen = 0; //2
      int saturationCount = 0;
      for( int i = 0; i < colorCrop.rows; i++ )
      {
        for(int j = 0; j < colorCrop.cols; j++ )
        {
          int H = colorCrop.at<cv::Vec3b>(i,j)[0];
          int S = colorCrop.at<cv::Vec3b>(i,j)[1];
          int V = colorCrop.at<cv::Vec3b>(i,j)[2];
          if (H < 26 && S > 200 && V > 240)
          {
               voteRed++;
          }
          else if (H < 42 && S > 50 && V > 240)
          {
               voteYellow++;
          }
          else if(H > 80 && H < 105 && S > 50 && V > 190)
          //else if(H > 80 && H < 105 && S > 170 && V > 220) //day
          {
              voteGreen++;
          }
          else
          {
              //other
          }
          if(S < 10)
          {
            saturationCount++;
          }
        }
      }
      voteYellow = voteYellow/2;
      if(voteRed > voteGreen && voteRed > voteYellow)
      {
        finalColor = 0;
        colorConfidence = (float)voteRed/(colorCrop.cols*colorCrop.rows-saturationCount);
      } 
      else if(voteGreen > voteRed && voteGreen > voteYellow)
      {
        finalColor = 2;
        colorConfidence = (float)voteGreen/(colorCrop.cols*colorCrop.rows-saturationCount);
      }
      else
      {
        finalColor = 1;
        colorConfidence = (float)voteYellow/(colorCrop.cols*colorCrop.rows-saturationCount);
      }

      /*
      // Using id of median intensity pixel as seed, center column
      cv::Mat sortedIndexes;
      cv::sortIdx(tempMat, sortedIndexes, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
      int medianRowIndex = sortedIndexes.at<uchar>(sortedIndexes.rows/2, sortedIndexes.cols/2);
      floodFill(intensityChannel.clone(), cv::Point(tempRect.x+tempRect.width/2, tempRect.y+medianRowIndex), cvScalar(255,0,0), &floodRect1, cvScalarAll(20), cvScalarAll(30), 8 | ( 255 << 8 ) | cv::FLOODFILL_FIXED_RANGE);
      */
      // alternative using id of max intensity pixel as seed

      // Ratio between floodFilledBB area and BBrect area if they intersect
      double max;
      cv::Point maxPoint;
      minMaxLoc(tempMat, NULL, &max, NULL, &maxPoint);
      floodFill(intensityChannel.clone(), cv::Point(tempRect.x+maxPoint.x, tempRect.y+maxPoint.y), cvScalar(255,0,0), &floodRect, cvScalarAll(40), cvScalarAll(30), 8 | ( 255 << 8 ) | cv::FLOODFILL_FIXED_RANGE);
      float floodArea = (float)floodRect.width*floodRect.height;
      float floodRatio;

      cv::Rect2d Rinter = tempRect & floodRect;     
      if (Rinter.area() > 0)
      {
        //rectangle(spotlightBBimage, Rinter, cv::Scalar( 255, 0, 255 ), 1, 4 );
        //cv::Rect2d Runion = preTrafficSignalCandidates[rectIndex1].rect2d | preTrafficSignalCandidates[rectIndex2].rect2d;
        float interesectArea = Rinter.width*Rinter.height;
        //float unionArea = Rinter.width*Rinter.height+tempRect.width*tempRect.height-interesectArea;

       if (Rinter.area() == tempRect.area())
       {
        if(floodArea < interesectArea)
        {
          floodRatio = (float)floodArea/interesectArea;
        }
        else
        {
          floodRatio = (float)interesectArea/floodArea;
        }
       }
       else if (Rinter.area() == floodRect.area())
       {
        if(tempRectArea < interesectArea)
        {
          floodRatio = (float)tempRectArea/interesectArea;
        }
        else
        {
          floodRatio = (float)interesectArea/tempRectArea;
        }
       }
       else
       {
        if(tempRectArea < interesectArea)
        {
          floodRatio = (float)tempRectArea/interesectArea;
        }
        else
        {
          floodRatio = (float)interesectArea/tempRectArea;
        }
       }
      }
      else
      {
       floodRatio = 0;
       //cout << "Non-overlapping Rectangles" << endl;
      }

#ifdef debug
      rectangle(presentationImage, tempRect, cv::Scalar( 0, 255, 255 ), 1, 4 );
      stringstream frameNumberStream;
      frameNumberStream << i;
      string frameNumberString = frameNumberStream.str();
      putText(presentationImage, frameNumberString, cv::Point(tempRect.x,tempRect.y), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(255,255,0), 3,3); 
      
      //rectangle(compareMat, floodRect, cv::Scalar( 255, 255, 0 ), 1, 4 );
      //cout << "Contour index: " << i << " solidity: " << solidity << " tempRectArea: " << tempRectArea << " floodArea: " << floodArea << " floodRatio: " << floodRatio << " rectRatio: " << rectRatio << endl;
      cout << "Contour nr: " << i << "--------------------" << i << endl;
      cout << "tempRect: " << tempRect << endl;
      cout << "finalColor: " << finalColor << endl;
      
      cout << "BBRatio: " << BBRatio << endl;
      cout << "floodRatio: " << floodRatio << endl;
      cout << "solidity: " << solidity << endl;
      cout << "meanIntensity: " << meanIntensity(0)/255 << endl;
      cout << "colorConfidence: " << colorConfidence << endl;
#endif
      //rectangle(compareMat, floodRect, cv::Scalar( 255, 255, 0 ), 1, 4 );
      //cout << "Contour index: " << i << " solidity: " << solidity << " tempRectArea: " << tempRectArea << " floodArea: " << floodArea << " floodRatio: " << floodRatio << " rectRatio: " << rectRatio << endl;
      ColorLight tempDetectedSpotLight;
      tempDetectedSpotLight.ROI = tempRect;
      tempDetectedSpotLight.color = finalColor;
      tempDetectedSpotLight.rectRatio = BBRatio;
      tempDetectedSpotLight.floodRatio = floodRatio;
      tempDetectedSpotLight.solidity = solidity;
      tempDetectedSpotLight.meanIntensity = meanIntensity(0)/255;
      tempDetectedSpotLight.colorConfidence = colorConfidence;
      detectedTLs.push_back(tempDetectedSpotLight); 

    }
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
