#include "backproject.h"

EdgeSegmentation::EdgeSegmentation(){} 

void EdgeSegmentation::findEdges(cv::Mat frame){
    //Canny detector
    cv::Mat detected_edges;
    Canny(frame, detected_edges, 190, 255, 3);
    imshow("detected_edges",detected_edges);
}
