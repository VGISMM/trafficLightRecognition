#include "spotLightDetection.h"

SpotLightDetection::SpotLightDetection(){} 

void SpotLightDetection::segmentSpotLights(){
  //cv::Mat compareMat = colorFrame.clone();
  //cvtColor(colorFrame, intensityChannel, CV_BGR2GRAY);
  /* //day
  cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
  cv::Mat sel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
  morphologyEx(intensityChannel, topHat, cv::MORPH_TOPHAT, sel1, cv::Point(-1,-1), 4 );
  erode(topHat, topHat, sel, cv::Point(-1, -1), 1, 1, 1);
  morphologyEx( topHat, topHat, cv::MORPH_OPEN, sel, cv::Point(-1,-1), 1 );
  */
  //cv::imshow("topHatBefore.png",intensityChannel);
  cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
  cv::Mat sel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));

  //morphologyEx(intensityChannel, topHat, cv::MORPH_TOPHAT, sel, cv::Point(-1,-1), 9 );
  morphologyEx(intensityChannel, topHat, cv::MORPH_TOPHAT, sel1, cv::Point(-1,-1), 4 );
  //cv::imwrite("tophat.png",topHat);

  //cv::imshow("topHat2.png",topHat);
  erode(topHat, topHat, sel1, cv::Point(-1, -1), 1, 1, 2);
  //morphologyEx( topHat, topHat, cv::MORPH_CLOSE, sel, cv::Point(-1,-1), 2 );
  morphologyEx( topHat, topHat, cv::MORPH_OPEN, sel, cv::Point(-1,-1), 1 );
  //cv::imshow("topHatAfter.png",topHat);
  //erode(topHat, topHat, sel, cv::Point(-1, -1), 1, 1, 1);
  erode(topHat, topHat, sel, cv::Point(-1, -1), 1, 1, 1);

  threshold( topHat, topHatBlobs, 25, 255,0 );
  dilate(topHatBlobs, topHatBlobs, sel1, cv::Point(-1, -1), 1, 1, 2);
  
  spotLightBlobAnalysis.calculateScores(topHatBlobs, topHat, colorFrame);
  //presentationImage = spotLightBlobAnalysis.presentationImage;
  //detectedTLs = spotLightBlobAnalysis.detectedTLs;
  //cv::imshow("presentationImage",spotLightBlobAnalysis.presentationImage);
}
