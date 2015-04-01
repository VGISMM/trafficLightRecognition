#include "edgeSegmentation.h"

EdgeSegmentation::EdgeSegmentation(){} 

void EdgeSegmentation::findEdges(cv::Mat frame){
  //Canny edge detector  
  equalizeHist(frame, frame); // improves edges
  Canny(frame, edgeMap, 80, 240, 3, 2); // threshold relationship should be x3
  //cv::resize(edgeMap,edgeMap,cv::Size(),0.5,0.5,CV_INTER_LINEAR);
  //imshow("detected_edges",edgeMap);

  // Reduce the noise so we avoid false circle detection
  //GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
  

  /// Apply the Hough Transform to find the circles
 // HoughCircles( frame, circles, CV_HOUGH_GRADIENT, 1, frame.rows/8, 240, 50, 0, 0 );

  
}
