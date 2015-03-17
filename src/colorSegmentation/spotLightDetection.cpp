#include "spotLightDetection.h"

SpotLightDetection::SpotLightDetection(){} 

void SpotLightDetection::segmentSpotLights(cv::Mat frame){
  //int morph_size = 1;
  //cv::Mat element = getStructuringElement( cv::MORPH_RECT, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
  cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
  morphologyEx(frame, topHat, cv::MORPH_TOPHAT, sel, cv::Point(-1,-1), 9 );
  morphologyEx( topHat, topHat, cv::MORPH_OPEN, sel, cv::Point(-1,-1), 2 );

  /*
  cv::vector<cv::vector<cv::Point>> contours; 
  cv::vector<cv::Vec4i> hierarchy;

  findContours( frame.clone(), contours, hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE  );
  cv::Mat out;
      int idx = 0;
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        cv::Scalar color( rand()&255, rand()&255, rand()&255 );
        drawContours( frame, contours, idx, color, CV_FILLED, 8, hierarchy );
    }
    imshow("out",frame);
  //threshold( topHat, topHat, 30, 255,0 );
  */
  cv::Mat cloneTop = topHat.clone();
  cv::resize(cloneTop,cloneTop,cv::Size(),0.5,0.5,CV_INTER_LINEAR);
  imshow("topHat",cloneTop);
}
