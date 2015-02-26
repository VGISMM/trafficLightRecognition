#include <iostream>
#include <vector_types.h>
#include <string>
#include <stdio.h>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "disparity/disparity.h"
#include "blobAnalysis/blobAnalysis.h"
#include "colorSegmentation/backproject.h"

using namespace std;

cv::VideoWriter output;

int main(int argc, char **argv) {
  Disparity Disparity;
  BlobAnalysis BlobAnalysis;
  Backproject Backproject;
  cv::VideoCapture capture(argv[1]);
  //output.open( "dispTest.avi", CV_FOURCC('M','J','P','G'), 16, cv::Size (imageWidth*2,imageHeight), true );

  cv::Mat img, imgLOI, imgROI, imgLGray, imgRGray, dispOutRLLR, dispOutFinished, dstYCrCb, dstLUV, dstCbUV;
  
  Backproject.init();
  while(1) {
    /** Get left and right stereo image pair **/
    capture >> img;

    if (img.data==NULL){
      //output.release();
      break;
      //capture.set(CV_CAP_PROP_POS_AVI_RATIO,0);
      //capture >> img;
    }
    imgLOI = img(cv::Rect(cv::Point(0,imageHeightOffset), cv::Size(imageWidth, imageHeight)));
    imgROI = img(cv::Rect(cv::Point(imageWidth,imageHeightOffset), cv::Size(imageWidth, imageHeight)));
   	

    // color space tests
    cv::Mat channelYCrCb[3];
    cv::Mat channelLuv[3];
    vector<cv::Mat> channels;
    cvtColor(imgLOI, dstYCrCb, CV_BGR2YCrCb); //YCrCb JPEG (or YCC)
    split(dstYCrCb, channelYCrCb);
    
    cvtColor(imgLOI, dstLUV, CV_BGR2Luv);
    split(dstLUV, channelLuv);
    channels.push_back(channelYCrCb[1]);
    channels.push_back(channelLuv[1]);
    channels.push_back(channelLuv[2]);

    merge(channels,dstCbUV); // combined
    //cvtColor(imgLOI, dstLUV, CV_BGR2Luv); // simple
    
    Backproject.backproject(dstCbUV);
    BlobAnalysis.blobRects.clear();
    BlobAnalysis.extractBlobs(Backproject.greenBP);
    BlobAnalysis.extractBlobs(Backproject.redBP);

    // test code for blobanalysis
    for( int i = 0; i< BlobAnalysis.blobRects.size(); i++ ){

		rectangle(imgLOI, BlobAnalysis.blobRects[i], cv::Scalar( 0, 55, 255 ), 2, 4 );
    }
	imshow("imgLOI",imgLOI);

    /* //gpu tests
    cv::cvtColor(imgLOI, imgLGray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(imgROI, imgRGray, cv::COLOR_BGR2GRAY);
    auto endgrab = chrono::steady_clock::now();
    Disparity.imgLOIGPU.upload(imgLGray);
    Disparity.imgROIGPU.upload(imgRGray);

    auto endupload = chrono::steady_clock::now();
      // Store the time difference between start and end
    auto diffupload = endupload - endgrab;
    //cout << "Upload: " << chrono::duration <double, milli> (diffupload).count() << " ms" << endl;

    Disparity.disparityImages();

    auto enddisp = chrono::steady_clock::now();
      // Store the time difference between start and end
    auto diffdisp = enddisp - endupload;
    cout << "Disparity Calculations: " << chrono::duration <double, milli> (diffdisp).count() << " ms" << endl;
    //Disparity.dispRLLRGPU.download(dispOutRLLR);
    Disparity.dispFinished.download(dispOutFinished);
    //imshow("dispOutRLLR",representGrayInColor(dispOutRLLR, 0, 255));

    cv::Mat combinedOutput=cv::Mat::zeros(imageHeight,imageWidth*2,imgLOI.type());
    imgLOI.copyTo(combinedOutput(cv::Rect(0,0,imageWidth,imageHeight)));
    representGrayInColor(dispOutFinished, 0, 255).copyTo(combinedOutput(cv::Rect(imageWidth,0,imageWidth,imageHeight)));
    output.write(combinedOutput);
    cv::resize(combinedOutput,combinedOutput,cv::Size(imageWidth,imageHeight/2),CV_INTER_LINEAR);
    //imshow("combinedOutput",combinedOutput);
	*/

    //imwrite("imgLOI.png", imgLOI); 
    cv::waitKey();
    int k = cv::waitKey(10);
    if (k=='q') {
      //imwrite("imgLOI.png", imgLOI); 
      //output.release();
      break;
    }
    if(k=='p') {
      cv::waitKey();
    }
  }
  return 0;
}
