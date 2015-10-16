#include "thresholdColor.h"
ThresholdColor::ThresholdColor(){}

void ThresholdColor::detectColors(){
  redTC = cv::Mat::zeros( cv::Size( colorFrame.cols, colorFrame.rows ), CV_8UC1 );
  greenTC = cv::Mat::zeros( cv::Size( colorFrame.cols, colorFrame.rows ), CV_8UC1 );
  yellowTC = cv::Mat::zeros( cv::Size( colorFrame.cols, colorFrame.rows ), CV_8UC1 );
  allTC = cv::Mat::zeros( cv::Size( colorFrame.cols, colorFrame.rows ), CV_8UC1 );

  thresholdFrameHSV();
  threshold( allTC, allTCBlobs, 10, 255, 0 );
  locateTrafficLights(allTCBlobs, intensity, 0);
  /*
  threshold( redTC, redTC, 15, 255, 0 );
  threshold( yellowTC, yellowTC, 15, 255, 0 );
  threshold( greenTC, greenTC, 15, 255, 0 );

  locateTrafficLights(redTC, 0);
  locateTrafficLights(yellowTC, 1);
  locateTrafficLights(greenTC, 2);
  */
  // Visualize the segmented lights directly
  
  std::vector<cv::Mat> TCchannels;
  TCchannels.push_back(yellowTC);
  TCchannels.push_back(greenTC);
  TCchannels.push_back(redTC);
  //TCchannels.push_back(allTC);

  merge(TCchannels, outputTC);
  
  /*
    for( int i = 0; i< detectedColorLights.size(); i++ )
  {
    rectangle(outBP, detectedColorLights[i].ROI, cv::Scalar( 0, 255, 255 ), 1, 4 );
  }
  cv::resize(outBP,outBP,cv::Size(),0.5,0.5,CV_INTER_LINEAR);
  imshow("TCChannels",outBP); 
  */
}

/*
cv::Mat thresholdColor::thresholdFrameLUV(cv::Mat frame){
	cvtColor(frame,converted,CV_BGR2Luv);
    cv::imshow("converted",converted);
    imgOut = cv::Mat::zeros( cv::Size( frame.cols, frame.rows ), CV_8UC3 );
    
    
    for( int i = 0; i < frame.rows; i++ )
    {
        for(int j = 0; j < frame.cols; j++ )
        {
            int L = converted.at<cv::Vec3b>(i,j)[0];
            int U = converted.at<cv::Vec3b>(i,j)[1];
            int V = converted.at<cv::Vec3b>(i,j)[2];

            if (L > 180 && && U > 130 && V > 180)
            {
                imgOut.at<cv::Vec3b>(i,j)[2] = 255;
            }
            else
            {
                imgOut.at<cv::Vec3b>(i,j)[0] = 255;
            }
        }
    }
    //cvtColor(imgOut,imgOut,CV_RGB2GRAY);
   // threshold( imgOut, EmGmmAll, 1, 255,0 );
    return imgOut;
} */

void ThresholdColor::thresholdFrameHSV()
{
  cvtColor(colorFrame, converted, CV_BGR2HSV);
#ifdef day
  for( int i = 0; i < colorFrame.rows; i++ )
  {
      for(int j = 0; j < colorFrame.cols; j++ )
      {
          int H = converted.at<cv::Vec3b>(i,j)[0];
          int S = converted.at<cv::Vec3b>(i,j)[1];
          int V = converted.at<cv::Vec3b>(i,j)[2];

          if (H < 35 && S > 140 && V > 150) //day
          {
              if (H < 26 && S > 200 && V > 240)
              {
                  redTC.at<uchar>(i,j) = 255; //red
                  allTC.at<uchar>(i,j) = 255; // all
              }
              else if (H < 35 && S > 80 && V > 240)
              {
                  yellowTC.at<uchar>(i,j) = 255; //yellow
                  allTC.at<uchar>(i,j) = 255; // all
              }
              else
              {
                  //yellowTC.at<uchar>(i,j) = 255; //yellow
              }    
          }
          else if(H > 80 && H < 105 && S > 170 && V > 220) //day
          {
              greenTC.at<uchar>(i,j) = 255; //green
              allTC.at<uchar>(i,j) = 255; // all
          }
      }
  }
#else //night
  for( int i = 0; i < colorFrame.rows; i++ )
  {
      for(int j = 0; j < colorFrame.cols; j++ )
      {
          int H = converted.at<cv::Vec3b>(i,j)[0];
          int S = converted.at<cv::Vec3b>(i,j)[1];
          int V = converted.at<cv::Vec3b>(i,j)[2];

          if (H < 42 && S > 80 && V > 240)
          {
              if (H < 26 && S > 200 && V > 240)
              {
                  redTC.at<uchar>(i,j) = 255; //red
                  allTC.at<uchar>(i,j) = 255; // all
              }
              else if (H < 42 && S > 80 && V > 240)
              {
                  yellowTC.at<uchar>(i,j) = 255; //yellow
                  allTC.at<uchar>(i,j) = 255; // all
              }
              else
              {
                  //yellowTC.at<uchar>(i,j) = 255; //yellow
              }    
          }
          else if(H > 80 && H < 105 && S > 150 && V > 190)
          {
              greenTC.at<uchar>(i,j) = 255; //green
              allTC.at<uchar>(i,j) = 255; // all
          }
      }
  }
#endif   
}

void ThresholdColor::locateTrafficLights(cv::Mat TCBlobs, cv::Mat intensityChannel, int color)
{
  cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
  cv::Mat sel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
  //erode(BPChannel, BPChannel, sel, cv::Point(-1, -1), 2, 1, 2);
 
  //dilate(TCChannel, TCChannel, sel, cv::Point(-1, -1), 2, 1, 1);
  //erode(TCChannel, TCChannel, sel, cv::Point(-1, -1), 1, 1, 1);
  morphologyEx( TCBlobs, TCBlobs, cv::MORPH_CLOSE, sel, cv::Point(-1,-1), 2 );
  morphologyEx( TCBlobs, TCBlobs, cv::MORPH_OPEN, sel, cv::Point(-1,-1), 2 );
  
  dilate(TCBlobs, TCBlobs, sel1, cv::Point(-1, -1), 1, 1, 4);

  colorThreshBlobAnalysis.calculateScores(TCBlobs, intensityChannel, colorFrame);
  

/*
  //erode(TCChannel, TCChannel, sel, cv::Point(-1, -1), 1, 1, 1);
  std::vector<std::vector<cv::Point>> contours; 
  std::vector<cv::Vec4i> hierarchy;
  findContours( TCChannel.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE); 
  cv::Rect tempRect, floodRect;
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
      float tempRectArea = (float)tempRect.width*(float)tempRect.height;
      cv::Mat tempMat1 = intensity(tempRect);
      cv::Scalar meanIntensity = mean(tempMat1);
      cv::Mat tempMat2 = TCChannel(tempRect);
      //cv::Scalar confidence = mean(tempMat2);
      
      // alternative using id of max intensity pixel as seed
      double max;
      //cv::Point maxPoint(tempRect.x+tempRect.width/2,tempRect.y+tempRect.height/2);
      //minMaxLoc(tempMat1, NULL, &max, NULL, &maxPoint);
      //maxPoint = ()
      floodFill(colorFrame.clone(), cv::Point(tempRect.x+tempRect.width/2,tempRect.y+tempRect.height/2), cvScalar(255,0,0), &floodRect, cvScalarAll(20), cvScalarAll(10), 8 | ( 255 << 8 ) | cv::FLOODFILL_FIXED_RANGE);
      rectangle(mything, floodRect, cv::Scalar( 0, 0, 255 ), 1, 4 );

      // Using id of median intensity pixel as seed, center column
      cv::Mat sortedIndexes;
      cv::sortIdx(tempMat, sortedIndexes, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
      int medianRowIndex = sortedIndexes.at<uchar>(sortedIndexes.rows/2, sortedIndexes.cols/2);

      // debug
      //int medianIntensity = intensity.at<uchar>(tempRect.y+medianRowIndex, tempRect.x+tempRect.width/2);
      //cv::Point medianPoint = cv::Point(medianRowIndex,sortedIndexes.cols/2);
      //cout << "value: " << medianIntensity << " " << medianPoint << endl;
      floodFill(intensity.clone(), cv::Point(tempRect.x+tempRect.width/2, tempRect.y+medianRowIndex), cvScalar(255,0,0), &floodRect, cvScalarAll(30), cvScalarAll(40), 8 | ( 255 << 8 ) | cv::FLOODFILL_FIXED_RANGE);

      cv::Mat colorMat = converted(tempRect);
      //cvtColor(colorMat, colorMat, CV_BGR2HSV);

      int finalColor;
      float confidence;
      int voteRed = 0; //0
      int voteYellow = 0; //1
      int voteGreen = 0; //2
      for( int i = 0; i < colorMat.rows; i++ )
      {
        for(int j = 0; j < colorMat.cols; j++ )
        {
          int H = colorMat.at<cv::Vec3b>(i,j)[0];
          int S = colorMat.at<cv::Vec3b>(i,j)[1];
          int V = colorMat.at<cv::Vec3b>(i,j)[2];
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
        }
      }

      if(voteRed > voteGreen && voteRed > voteYellow)
      {
        finalColor = 0;
        confidence = (float)voteRed/(colorMat.cols*colorMat.rows);
      } 
      else if(voteGreen > voteRed && voteGreen > voteYellow)
      {
        finalColor = 2;
        confidence = (float)voteGreen/(colorMat.cols*colorMat.rows);
      }
      else
      {
        finalColor = 1;
        confidence = (float)voteYellow/(colorMat.cols*colorMat.rows);
      }

      float floodArea = (float)floodRect.width*(float)floodRect.height;
      float floodRatio;
      if(floodArea < tempRectArea)
      {
        floodRatio = floodArea/tempRectArea;
      }
      else
      {
        floodRatio = tempRectArea/floodArea;
      }
      //if(floodRatio > 0.1)
      //{
        rectangle(mything, tempRect, cv::Scalar( 0, 255, 255 ), 1, 4 );
        //cout << "Contour index: " << i << " confidence: " << confidence[0]/255 << " tempRectArea: " << tempRectArea << " floodArea: " << floodArea << " floodRatio: " << floodRatio << " rectRatio: " << rectRatio << endl;
        ColorLight tempDetectedColorLight;
        tempDetectedColorLight.color = finalColor;
        tempDetectedColorLight.ROI = tempRect;
        tempDetectedColorLight.rectRatio = rectRatio;
        tempDetectedColorLight.floodRatio = floodRatio;
        tempDetectedColorLight.meanIntensity = meanIntensity[0]/255;
        tempDetectedColorLight.confidence = confidence; //[0]/255;
        detectedColorLights.push_back(tempDetectedColorLight); 
      //}
    }
  }
  */
}
