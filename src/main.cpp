#include <iostream>

#include <string>
#include <stdio.h>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "disparity/disparity.h"
#include "blobAnalysis/blobAnalysis.h"
#include "pressentation/pressentation.h"
#ifdef backdef
#include "colorSegmentation/backproject.h"
#endif
#ifdef emGMMdef
#include "colorSegmentation/emGMM.h"
#endif
#include "pointCloud/pointCloud.h"
#include "trafficSignal/trafficSignal.h"

using namespace std;

cv::VideoWriter output;

int main(int argc, char **argv) {
  Disparity Disparity;
  BlobAnalysis BlobAnalysis;
  #ifdef backdef
  Backproject Backproject;
  #endif
  #ifdef emGMMdef
  emGMM emGMM;
  #endif
  PointCloud PointCloud;
  Pressentation Pressentation;
  cv::vector<TrafficSignal> frameTrafficSignals;
  cv::VideoCapture capture(argv[1]);
  output.open( "output/out.avi", CV_FOURCC('M','J','P','G'), 16, cv::Size (IMAGEWIDTH*2,IMAGEHEIGHT), true );

  cv::Mat img, imgLOI, imgROI, imgLGray, imgRGray, dispOutRLLR, dispOutFinished, dstYCrCb, dstLUV, dstCbUV;
  
  #ifdef backdef
  Backproject.init();
  #endif
  #ifdef emGMMdef
  emGMM.init();
  #endif 

  while(1) {
    /** Get left and right stereo image pair **/
    capture >> img;

    if (img.data==NULL){
      output.release();
      break;
      //capture.set(CV_CAP_PROP_POS_AVI_RATIO,0);
      //capture >> img;
    }
    imgLOI = img(cv::Rect(cv::Point(0,IMAGEHEIGHTOFFSET), cv::Size(IMAGEWIDTH, IMAGEHEIGHT)));
    imgROI = img(cv::Rect(cv::Point(IMAGEWIDTH,IMAGEHEIGHTOFFSET), cv::Size(IMAGEWIDTH, IMAGEHEIGHT)));
   	
    ///* //gpu tests
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
    //imshow("dispOutFinished",Pressentation.representGrayInColor(dispOutFinished, 0, 255));

    // Clear vectors for every frame
    BlobAnalysis.blobRects.clear();
    BlobAnalysis.biggerBlobRects.clear();
    frameTrafficSignals.clear();

    #ifdef backdef
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

    BlobAnalysis.extractBlobs(Backproject.greenBP, imgLOI);
    BlobAnalysis.extractBlobs(Backproject.redBP, imgLOI);
    #endif

        #ifdef emGMMdef
    emGMM.predictFrame(imgLOI);
    BlobAnalysis.extractBlobs(emGMM.EmGmmAll, imgLOI);
    #endif
    // test code for blobanalysis

    PointCloud.init(dispOutFinished, imgLOI);
    for( int i = 0; i< BlobAnalysis.biggerBlobRects.size(); i++ ){
      if (PointCloud.dispToXYZRGB(BlobAnalysis.biggerBlobRects[i]))
      {
        TrafficSignal tempTrafficSignal;
        tempTrafficSignal.rect2d = BlobAnalysis.biggerBlobRects[i];
        tempTrafficSignal.trafficSignalPosition = PointCloud.clusterFront3Dpoint;
        frameTrafficSignals.push_back(tempTrafficSignal);
      }

    }

    for( int i = 0; i< frameTrafficSignals.size(); i++ ){
      rectangle(PointCloud.biggerLOI, frameTrafficSignals[i].rect2d, cv::Scalar( 0, 55, 255 ), 2, 4 );
    }

    // Pressentation stuff
	  cv::resize(PointCloud.biggerLOI,imgLOI,cv::Size(IMAGEWIDTH,IMAGEHEIGHT),CV_INTER_LINEAR);   
    cv::Mat combinedOutput=cv::Mat::zeros(IMAGEHEIGHT,IMAGEWIDTH*2,imgLOI.type());
    imgLOI.copyTo(combinedOutput(cv::Rect(0,0,IMAGEWIDTH,IMAGEHEIGHT)));
    Pressentation.representGrayInColor(dispOutFinished, 0, 255).copyTo(combinedOutput(cv::Rect(IMAGEWIDTH,0,IMAGEWIDTH,IMAGEHEIGHT)));
    output.write(combinedOutput);
    cv::resize(combinedOutput,combinedOutput,cv::Size(IMAGEWIDTH,IMAGEHEIGHT/2),CV_INTER_LINEAR);
    imshow("combinedOutput",combinedOutput);
    
    //imwrite("imgLOI.png", imgLOI); 
    cv::waitKey();
    int k = cv::waitKey(10);
    if (k=='q') {
      //imwrite("imgLOI.png", imgLOI); 
      output.release();
      break;
    }
    if(k=='p') {
      cv::waitKey();
    }
  }
  return 0;
}
