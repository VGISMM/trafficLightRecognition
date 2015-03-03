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
#include "ransac/ransac.h"
#include "tracking/kalmanfilter2d.h"
#include "trafficSignal/trafficSignal.h"

using namespace std;

cv::VideoWriter output;

float thresholdOffset = -5.1;

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
  Ransac Ransac;
  Kalman Kalman;
  cv::vector<TrafficSignal> frameTrafficSignals;
  cv::VideoCapture capture(argv[1]);
  output.open( "output/out.avi", CV_FOURCC('M','J','P','G'), 16, cv::Size (IMAGEWIDTH*2+255,IMAGEHEIGHT), true );

  cv::Mat img, imgLOI, imgROI;  // Input images  , imgLGray, imgRGray; 
  cv::Mat dispOutFinished, dispLower, vdispout; // Output images
  cv::Mat dstYCrCb, dstLUV, dstCbUV; // Colorspaces
  
  #ifdef backdef
  Backproject.init();
  #endif
  #ifdef emGMMdef
  emGMM.init();
  #endif

  bool first=true;

  while(1) {
//-----------------------------------Input handling-------------------------------------------------
    // Get left and right stereo image pair 
    capture >> img;
    if (img.data==NULL){
      output.release();
      break;
      //capture.set(CV_CAP_PROP_POS_AVI_RATIO,0);
      //capture >> img;
    }
    imgLOI = img(cv::Rect(cv::Point(0,0), cv::Size(IMAGEWIDTH, IMAGEHEIGHT)));
    imgROI = img(cv::Rect(cv::Point(IMAGEWIDTH,0), cv::Size(IMAGEWIDTH, IMAGEHEIGHT)));
//-----------------------------------Colorspace conversion-------------------------------------------
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
//-----------------------------------Disp calculations-----------------------------------------------
    //gpu disp upper image
    //cv::cvtColor(imgLOI, imgLGray, cv::COLOR_BGR2GRAY);
    //cv::cvtColor(imgROI, imgRGray, cv::COLOR_BGR2GRAY);
    Disparity.imgLOIGPU.upload(imgLOI);
    Disparity.imgROIGPU.upload(imgROI);
    auto endupload = chrono::steady_clock::now();
    Disparity.GPUBMdisparity();
    auto enddisp = chrono::steady_clock::now();
    auto diffdisp = enddisp - endupload;
    cout << "Disparity Calculations: " << chrono::duration <double, milli> (diffdisp).count() << " ms" << endl;
    Disparity.dispFinished.download(dispOutFinished);
    /*
    //SGBM alternative
    Disparity.imgL = imgLOI.clone();
    Disparity.imgR = imgROI.clone();
    Disparity.SGBMdisparity();
    dispOutFinished = Disparity.dispRLLR;
    */
//-------------------------------Road Surface localization---------------------------------------------
    Disparity.generateVdispCPU(dispOutFinished);
    vdispout = Disparity.vdispCPU.clone();
    /*
    // GPU vdisp
    Disparity.generateVdisp(dispLower);
    imshow("vdispout",Pressentation.representGrayInColor(Disparity.vdisp, 0, 255));
    imshow("vdispCPU",Pressentation.representGrayInColor(Disparity.vdispCPU, 0, 255));
    */
    Ransac.runRansac(vdispout);

    // Initilize Kalman at first frame
    if (first)
    {
      Kalman.initKalman((float)Ransac.slope,(float)Ransac.intersection);
      first = false;
    }
    Kalman.kalmanCorrect((float)Ransac.slope, (float)Ransac.intersection);
    Kalman.kalmanPredict();
    
    // Present the found road surface on the vdisp 
    circle(vdispout, Ransac.ransacPoint1, 3, cvScalar(150,150,150), 2, 8, 0);
    circle(vdispout, Ransac.ransacPoint2, 3, cvScalar(220,220,220), 2, 8, 0);
    line(vdispout, cv::Point((double)20,(double)(Ransac.slope*20+Ransac.intersection)-thresholdOffset),  cv::Point((double)80,(double)(Ransac.slope*80+Ransac.intersection)-thresholdOffset), cvScalar(155,155,155), 1, CV_AA);
    line(vdispout, cv::Point((double)20,(double)(Kalman.estimated.at<float>(0)*20+Kalman.estimated.at<float>(1))-thresholdOffset),  cv::Point((double)80,(double)(Kalman.estimated.at<float>(0)*80+Kalman.estimated.at<float>(1))-thresholdOffset), cvScalar(255,255,255), 1, CV_AA);
    // Threshold the disparity image just below the found road surface
    Disparity.vDispThresholdedImage(dispOutFinished, Kalman.estimated.at<float>(0), Kalman.estimated.at<float>(1), thresholdOffset);
    // Initiallize the pointcloud obejct with depth and color info
    PointCloud.init(Disparity.obstacleImage, dstCbUV);
    PointCloud.findRoadSurfaceCoefficients();
//-----------------------------------Traffic light localization-----------------------------------------------
   // Clear vectors for every frame
    BlobAnalysis.blobRects.clear();
    BlobAnalysis.biggerBlobRects.clear();
    frameTrafficSignals.clear();

    #ifdef backdef
    Backproject.backproject(dstCbUV);
    BlobAnalysis.extractBlobs(Backproject.greenBP(cv::Rect(cv::Point(BOXSIZE,BOXSIZE), cv::Size(IMAGEWIDTH-BOXSIZE, IMAGEHEIGHT-BOXSIZE))), imgLOI(cv::Rect(cv::Point(BOXSIZE,BOXSIZE), cv::Size(IMAGEWIDTH-BOXSIZE, IMAGEHEIGHT-BOXSIZE))));
    BlobAnalysis.extractBlobs(Backproject.redBP(cv::Rect(cv::Point(BOXSIZE,BOXSIZE), cv::Size(IMAGEWIDTH-BOXSIZE, IMAGEHEIGHT-BOXSIZE))), imgLOI(cv::Rect(cv::Point(BOXSIZE,BOXSIZE), cv::Size(IMAGEWIDTH-BOXSIZE, IMAGEHEIGHT-BOXSIZE))));
    #endif

    #ifdef emGMMdef
    emGMM.predictFrame(imgLOI);
    BlobAnalysis.extractBlobs(emGMM.EmGmmAll(cv::Rect(cv::Point(BOXSIZE,BOXSIZE), cv::Size(IMAGEWIDTH-BOXSIZE, IMAGEHEIGHT-BOXSIZE))), imgLOI(cv::Rect(cv::Point(BOXSIZE,BOXSIZE), cv::Size(IMAGEWIDTH-BOXSIZE, IMAGEHEIGHT-BOXSIZE))));
    #endif
    
    // Create vector with all accepted light signals
    for( int i = 0; i< BlobAnalysis.biggerBlobRects.size(); i++ ){
      if (PointCloud.projectRegionToPointCloud(BlobAnalysis.biggerBlobRects[i]))
      {
        TrafficSignal tempTrafficSignal;
        tempTrafficSignal.rect2d = BlobAnalysis.biggerBlobRects[i]; // should not be used
        tempTrafficSignal.trafficSignalPosition = PointCloud.clusterFront3Dpoint;
        tempTrafficSignal.heightAboveRoad = PointCloud.pointDistanceFromPlane; 
        tempTrafficSignal.trafficSignalPlanePoint = PointCloud.pointProjected2Plane;
        tempTrafficSignal.findTrafficSignalsImageCoordiantes();
        frameTrafficSignals.push_back(tempTrafficSignal);
      }
    }
    
//-----------------------------------Pressentation stuff-----------------------------------------------
    for( int i = 0; i< frameTrafficSignals.size(); i++ ){
      rectangle(imgLOI, cv::Rect(cv::Point(frameTrafficSignals[i].trafficSignalPosition2D.x-BOXSIZE,frameTrafficSignals[i].trafficSignalPosition2D.y-BOXSIZE), cv::Size(BOXSIZE*2, BOXSIZE*2)), cv::Scalar( 0, 55, 255 ), 2, 4 );
      //circle(PointCloud.biggerLOI, cv::Point(frameTrafficSignals[i].trafficSignalPosition2D.x, frameTrafficSignals[i].trafficSignalPosition2D.y), 4, cvScalar(255,150,150), 2, 8, 4);
      line(imgLOI, cv::Point( frameTrafficSignals[i].trafficSignalPlanePoint2D.x, frameTrafficSignals[i].trafficSignalPlanePoint2D.y ), cv::Point( frameTrafficSignals[i].trafficSignalPosition2D.x, frameTrafficSignals[i].trafficSignalPosition2D.y), cv::Scalar( 110, 220, 0 ),  2, 8 );

      string pngPath;
      stringstream pngfilename;   
      pngfilename << setprecision(2) << " d: " << frameTrafficSignals[i].trafficSignalPosition.z << " h: " << frameTrafficSignals[i].heightAboveRoad;
      pngPath = pngfilename.str();
      cv::putText(imgLOI, pngPath, cv::Point(frameTrafficSignals[i].rect2d.x,frameTrafficSignals[i].rect2d.y), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar( 0, 55, 255 ), 2,3);
    }

    cv::Mat combinedOutput=cv::Mat::zeros(IMAGEHEIGHT,IMAGEWIDTH*2+255,imgLOI.type());
    imgLOI.copyTo(combinedOutput(cv::Rect(0,0,IMAGEWIDTH,IMAGEHEIGHT)));
    Pressentation.representGrayInColor(Disparity.obstacleImage, 0, 255).copyTo(combinedOutput(cv::Rect(IMAGEWIDTH,0,IMAGEWIDTH,IMAGEHEIGHT)));
    Pressentation.representGrayInColor(vdispout, 0, 255).copyTo(combinedOutput(cv::Rect(IMAGEWIDTH*2,0,255,IMAGEHEIGHT)));
    output.write(combinedOutput);
    cv::resize(combinedOutput,combinedOutput,cv::Size(IMAGEWIDTH+128,IMAGEHEIGHT/2),CV_INTER_LINEAR);
    imshow("combinedOutput",combinedOutput);
    
    //cv::waitKey();
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
