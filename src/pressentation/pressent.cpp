#include "Pressent.h"
using namespace std;
Pressent::Pressent(){}

cv::Mat Pressent::representGrayInColor(cv::Mat gray, int min, int max) 
{
   // double min = 0;
   // double max = 255;
    //cv::minMaxIdx(grayDisp, &min, &max);
    cv::Mat adjMap;
    // expand your range to 0..255. Similar to histEq();
    gray.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min); 
    cv::Mat falseColorsMap;
    applyColorMap(adjMap, falseColorsMap, 2);
    return falseColorsMap;
}

