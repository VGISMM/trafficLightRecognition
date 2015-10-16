#include "backproject.h"

Backproject::Backproject(){} 

// Creating color histograms for backprojection
void Backproject::init(){
  const int channels[] = { 1, 2 };
  float range[] = { 0, 256 };
  const float *ranges[] = { range, range };
  const int histSize[] = { 256, 256 };

  char path[200];
  cv::Mat greenTraining = cv::Mat::zeros(20,20*numberOfTrainingSamples,CV_8UC3);
  cv::Mat redTraining = cv::Mat::zeros(20,20*numberOfTrainingSamples,CV_8UC3);
  cv::Mat yellowTraining = cv::Mat::zeros(20,20*numberOfTrainingSamples,CV_8UC3);

#ifdef day
  for(int trainingImageIndex=0;trainingImageIndex<numberOfTrainingSamples;trainingImageIndex++)
  {
    sprintf(path, "data/greenTraining2/%i.png", trainingImageIndex);
    cv::imread(path,1).copyTo(greenTraining(cv::Rect(20*trainingImageIndex,0,20,20)));
  }

  for(int trainingImageIndex=0;trainingImageIndex<numberOfTrainingSamples;trainingImageIndex++)
  {
    sprintf(path, "data/redTraining2/%i.png", trainingImageIndex);
    cv::imread(path,1).copyTo(redTraining(cv::Rect(20*trainingImageIndex,0,20,20)));
  }

  for(int trainingImageIndex=0;trainingImageIndex<numberOfTrainingSamples;trainingImageIndex++)
  {
    sprintf(path, "data/yellowNight/%i.png", trainingImageIndex);
    cv::imread(path,1).copyTo(yellowTraining(cv::Rect(20*trainingImageIndex,0,20,20)));
  }
#else
  for(int trainingImageIndex=0;trainingImageIndex<numberOfTrainingSamples;trainingImageIndex++)
  {
    sprintf(path, "data/greenNight/%i.png", trainingImageIndex);
    cv::imread(path,1).copyTo(greenTraining(cv::Rect(20*trainingImageIndex,0,20,20)));
  }

  for(int trainingImageIndex=0;trainingImageIndex<numberOfTrainingSamples;trainingImageIndex++)
  {
    sprintf(path, "data/redNight/%i.png", trainingImageIndex);
    cv::imread(path,1).copyTo(redTraining(cv::Rect(20*trainingImageIndex,0,20,20)));
  }

  for(int trainingImageIndex=0;trainingImageIndex<numberOfTrainingSamples;trainingImageIndex++)
  {
    sprintf(path, "data/yellowNight/%i.png", trainingImageIndex);
    cv::imread(path,1).copyTo(yellowTraining(cv::Rect(20*trainingImageIndex,0,20,20)));
  }
#endif
 
  cvtColor(greenTraining, greenTraining, CV_BGR2Luv);
  calcHist(&greenTraining, 1, channels, cv::noArray(), greenHistogram, 2, histSize, ranges, true, false);
  normalize(greenHistogram, greenHistogram, 0, 255, cv::NORM_MINMAX); 

  cvtColor(redTraining, redTraining, CV_BGR2Luv);
  calcHist(&redTraining, 1, channels, cv::noArray(), redHistogram, 2, histSize, ranges, true, false);
  normalize(redHistogram, redHistogram, 0, 255, cv::NORM_MINMAX);

  cvtColor(yellowTraining, yellowTraining, CV_BGR2Luv);
  calcHist(&yellowTraining, 1, channels, cv::noArray(), yellowHistogram, 2, histSize, ranges, true, false);
  normalize(yellowHistogram, yellowHistogram, 0, 255, cv::NORM_MINMAX); 

  //imshow( "redTraining", redTraining );
  imwrite("data/histogram/greenHistImg.png", showHistogram(greenHistogram));
  imwrite("data/histogram/redHistImg.png", showHistogram(redHistogram));
  imwrite("data/histogram/yellowHistImg.png", showHistogram(yellowHistogram));
}

void Backproject::detectColors(){
  const int channels[] = { 1, 2 };
  float range[] = { 0, 256 };
  const float *ranges[] = { range, range };

  /*
  cv::Mat junk = frame(cv::Rect(cv::Point(0,0), cv::Size(IMAGEWIDTH, IMAGEHEIGHT/6)));
  //calcHist(&objectROI, 1, channels, noArray(), objectHistogram, 2, histSize, ranges, true, false);
  // Global color distribution with cumulative histogram
  const int histSize[] = { 256,256 };
  calcHist(&junk, 1, channels, cv::noArray(), globalHistogram, 2, histSize, ranges, true, true);
  // Boosting: Divide conditional probabilities in object area by a priori probabilities of colors
  for (int y = 0; y < greenHistogram.rows; y++) {
    for (int x = 0; x < greenHistogram.cols; x++) {
      greenHistogram.at<float>(y, x) /= globalHistogram.at<float>(y, x);
      redHistogram.at<float>(y, x) /= globalHistogram.at<float>(y, x);
    }
  }
  imwrite("data/greenTraining/globalHistogram.png", showHistogram(globalHistogram));
  imwrite("data/redTraining/redHistImg1.png", showHistogram(redHistogram));
  */
  //imshow("intensity",intensity);

  cvtColor(colorFrame, LUVFrame, CV_BGR2Luv);
  calcBackProject(&LUVFrame, 1, channels, redHistogram, redBP, ranges);
  calcBackProject(&LUVFrame, 1, channels, yellowHistogram, yellowBP, ranges);
  calcBackProject(&LUVFrame, 1, channels, greenHistogram, greenBP, ranges);
cv::Mat greenBlobs, redBlobs, yellowBlobs;
#ifdef day
  threshold( yellowBP, yellowBlobs, 155, 255, 0 );
  threshold( greenBP, greenBlobs, 85, 255, 0 );
  threshold( redBP, redBlobs, 65, 255, 0 );
#else
  threshold( yellowBP, yellowBlobs, 125, 255, 0 );
  threshold( greenBP, greenBlobs, 90, 255, 0 );
  threshold( redBP, redBlobs, 60, 255, 0 );
#endif
  
  locateTrafficLights(redBP, redBlobs, 0);
  locateTrafficLights(yellowBP, yellowBlobs, 1);
  locateTrafficLights(greenBP, greenBlobs, 2);

  // Visualize the segmented lights directly
  std::vector<cv::Mat> BPchannels;
  
  BPchannels.push_back(yellowBlobs);
  BPchannels.push_back(greenBlobs);
  BPchannels.push_back(redBlobs);

  merge(BPchannels, outBP);


  /*
    for( int i = 0; i< detectedColorLights.size(); i++ )
  {
    rectangle(outBP, detectedColorLights[i].ROI, cv::Scalar( 0, 255, 255 ), 1, 4 );
  }
  cv::resize(outBP,outBP,cv::Size(),0.5,0.5,CV_INTER_LINEAR);
  imshow("BPchannels",outBP); 
  */
}

void Backproject::locateTrafficLights(cv::Mat BPChannel, cv::Mat blobChannel, int color){
  //threshold( BPChannel, BPChannel, 5, 255, 0 );
  cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
  cv::Mat sel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
  //erode(BPChannel, BPChannel, sel, cv::Point(-1, -1), 2, 1, 2);
  dilate(blobChannel, blobChannel, sel, cv::Point(-1, -1), 1, 1, 3);
  //dilate(BPChannel, BPChannel, sel, cv::Point(-1, -1), 2, 1, 2);
  
  morphologyEx( blobChannel, blobChannel, cv::MORPH_CLOSE, sel1, cv::Point(-1,-1), 2 );
  morphologyEx( blobChannel, blobChannel, cv::MORPH_OPEN, sel, cv::Point(-1,-1), 1 );
  
  dilate(blobChannel, blobChannel, sel1, cv::Point(-1, -1), 1, 1, 2);

  backprojBlobAnalysis.calculateScores(blobChannel, BPChannel, colorFrame);
  //erode(BPChannel, BPChannel, sel, cv::Point(-1, -1), 2, 1, 1);
/*
  std::vector<std::vector<cv::Point>> contours; 
  std::vector<cv::Vec4i> hierarchy;
  findContours( BPChannel.clone(), contours, hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE  ); 

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
      cv::Mat tempMat2 = BPChannel(tempRect);
      cv::Scalar confidence = mean(tempMat2);
      
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
        tempDetectedColorLight.color = color;
        tempDetectedColorLight.ROI = tempRect;
        tempDetectedColorLight.rectRatio = rectRatio;
        tempDetectedColorLight.floodRatio = floodRatio;
        tempDetectedColorLight.meanIntensity = meanIntensity[0]/255;
        tempDetectedColorLight.confidence = confidence[0]/255;
        detectedColorLights.push_back(tempDetectedColorLight); 
      //}
    }
  }
  */
}


cv::Mat Backproject::showHistogram(cv::Mat histogram){
  // Visualize histograms
  int w = 256; int h = 256;
  int bin_w = cvRound( (double) w / 256 );
  cv::Mat histImg = cv::Mat::zeros(w, h, CV_8UC3);
  cv::reduce(histogram, objectHistogram1, 0, CV_REDUCE_SUM, -1);
  normalize(objectHistogram1, objectHistogram1, 0, 255, cv::NORM_MINMAX); 
  cv::reduce(histogram, objectHistogram2, 1, CV_REDUCE_SUM, -1);
  normalize(objectHistogram2, objectHistogram2, 0, 255, cv::NORM_MINMAX); 
  for( int i = 1; i < 256; i++ )
  {
    line( histImg, cv::Point( bin_w*(i-1), h - cvRound(objectHistogram1.at<float>(i-1)) ),
                        cv::Point( bin_w*(i), h - cvRound(objectHistogram1.at<float>(i)) ),
                        cv::Scalar( 0, 255, 0), 1, 8, 0  );
    line( histImg, cv::Point( bin_w*(i-1), h - cvRound(objectHistogram2.at<float>(i-1)) ),
                        cv::Point( bin_w*(i), h - cvRound(objectHistogram2.at<float>(i)) ),
                        cv::Scalar( 255, 0, 0), 1, 8, 0  );
  }
 return histImg; 
}
