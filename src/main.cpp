#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "disparity/disparity.h"
//#include "blobAnalysis/blobAnalysis.h"
#include "pressentation/pressentation.h"
#include "candidateEvaluator/candidateEvaluator.h"
#ifdef backdef
#include "colorSegmentation/backproject.h"
#endif
#ifdef emGMMdef
#include "colorSegmentation/emGMM.h"
#endif
#include "colorSegmentation/spotLightDetection.h"
#include "shapeSegmentation/edgeSegmentation.h"
#include <opencv2/opencv.hpp>
#include "pointCloud/pointCloud.h"
#include "ransac/ransac.h"
#include "tracking/kalmanfilter2d.h"
#include "trafficSignal/trafficSignal.h"

using namespace std;

cv::VideoWriter output;

float thresholdOffset = -5.1;

int main(int argc, char **argv) {
  Disparity Disparity;
  EdgeSegmentation EdgeSegmentation;
  SpotLightDetection SpotLightDetection;
  //BlobAnalysis BlobAnalysis;
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
  CandidateEvaluator CandidateEvaluator;
  
  cv::VideoCapture capture(argv[1]);
  output.open( "output/out.avi", CV_FOURCC('M','J','P','G'), 16, cv::Size (IMAGEWIDTH*2+255,IMAGEHEIGHT), true );

  cv::Mat img, imgLOI, imgROI, imgPressentation;  // Input images  , imgLGray, imgRGray; 
  cv::Mat dispOutFinished, dispLower, dispUpper, vdispout, spotlightOut, colorOut; // Output images
  cv::Mat dstYCrCb, dstLUV; // Colorspaces
  
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
    //vector<cv::Mat> customChannels;
    //cv::Mat channelYCrCb[3];
    //cvtColor(imgLOI, dstYCrCb, CV_BGR2YCrCb); //YCrCb JPEG (or YCC)
    //split(dstYCrCb, channelYCrCb);
    
    cv::Mat colorChannels[3];
    cvtColor(imgLOI, dstLUV, CV_BGR2Luv);
    split(dstLUV, colorChannels);

    //customChannels.push_back(channelLuv[0]);
    //customChannels.push_back(channelLuv[1]);
    //customChannels.push_back(channelLuv[2]);
    //merge(customChannels,dstLUV); // combined
    
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
    Disparity.upperDispFinished.download(dispUpper);
    Disparity.lowerDispFinished.download(dispLower);
    dispOutFinished=cv::Mat::zeros(IMAGEHEIGHT,IMAGEWIDTH,dispUpper.type());
    dispUpper.copyTo(dispOutFinished(cv::Rect(0,0,IMAGEWIDTH,IMAGEHEIGHT/2)));
    dispLower.copyTo(dispOutFinished(cv::Rect(0,IMAGEHEIGHT/2,IMAGEWIDTH,IMAGEHEIGHT/2)));
    
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
    PointCloud.init(Disparity.obstacleImage, imgLOI);
    PointCloud.findRoadSurfaceCoefficients();
//-----------------------------------Traffic light Segmentation-----------------------------------------------
   
    // Clear vectors for every frame

    SpotLightDetection.detectedSpotLights.clear();
    Backproject.detectedColorLights.clear();
    // input must be grayscale
    EdgeSegmentation.findEdges(colorChannels[0]);
    SpotLightDetection.segmentSpotLights(colorChannels[0],imgLOI.clone()); // color only for debug
    spotlightOut = SpotLightDetection.topHat;
    #ifdef backdef
    Backproject.colorFrame = dstLUV;
    Backproject.intensity = colorChannels[0];
    Backproject.backproject();
    //colorOut = Backproject.outBP;
    //BlobAnalysis.extractBlobs(Backproject.greenBP(cv::Rect(cv::Point(BOXSIZE,BOXSIZE), cv::Size(IMAGEWIDTH-BOXSIZE, IMAGEHEIGHT-BOXSIZE))), imgLOI(cv::Rect(cv::Point(BOXSIZE,BOXSIZE), cv::Size(IMAGEWIDTH-BOXSIZE, IMAGEHEIGHT-BOXSIZE))));
    #endif
    #ifdef emGMMdef
    emGMM.predictFrame(dstLUV);
    //colorOut = emGMM.EmGmmAll;
    //BlobAnalysis.extractBlobs(emGMM.EmGmmAll(cv::Rect(cv::Point(BOXSIZE,BOXSIZE), cv::Size(IMAGEWIDTH-BOXSIZE, IMAGEHEIGHT-BOXSIZE))), imgLOI(cv::Rect(cv::Point(BOXSIZE,BOXSIZE), cv::Size(IMAGEWIDTH-BOXSIZE, IMAGEHEIGHT-BOXSIZE))));
    #endif

    CandidateEvaluator.collectCandidates(SpotLightDetection.detectedSpotLights, Backproject.detectedColorLights);
    CandidateEvaluator.stereoVisionEvaluation(PointCloud);
   
//-----------------------------------Pressentation stuff-----------------------------------------------
    imgPressentation = imgLOI.clone();
    for( int i = 0; i< CandidateEvaluator.trafficSignalCandidates.size(); i++ ){
    	float combinedConfidence = CandidateEvaluator.trafficSignalCandidates[i].spotLightConfidence+CandidateEvaluator.trafficSignalCandidates[i].colorLightConfidence;
   	  if (combinedConfidence > 0.3)
   	  {
   	  	// Red and light red are color detections, Blue and light blue are spotlight detections, Other are detections by both
   	  	//rectangle(imgPressentation, cv::Rect(cv::Point(trafficSignalCandidates[i].trafficSignalPosition2D.x,trafficSignalCandidates[i].trafficSignalPosition2D.y), cv::Size(BOXSIZE*2, BOXSIZE*2)), cv::Scalar( 0, 55, 255 ), 2, 4 );
	      circle(imgPressentation, cv::Point(CandidateEvaluator.trafficSignalCandidates[i].trafficSignalPosition2D.x, CandidateEvaluator.trafficSignalCandidates[i].trafficSignalPosition2D.y), (1+combinedConfidence)*(1+combinedConfidence)*10, CandidateEvaluator.trafficSignalCandidates[i].trafficLightColor, 2, 8, 0);
	      line(imgPressentation, cv::Point(CandidateEvaluator.trafficSignalCandidates[i].trafficSignalPlanePoint2D.x, CandidateEvaluator.trafficSignalCandidates[i].trafficSignalPlanePoint2D.y), cv::Point( CandidateEvaluator.trafficSignalCandidates[i].trafficSignalPosition2D.x, CandidateEvaluator.trafficSignalCandidates[i].trafficSignalPosition2D.y), CandidateEvaluator.trafficSignalCandidates[i].trafficLightColor,  2, 8 );

	      string pngPath;
	      stringstream pngfilename;   
	      pngfilename << setprecision(2) << " d: " << CandidateEvaluator.trafficSignalCandidates[i].trafficSignalPosition.z << " h: " << CandidateEvaluator.trafficSignalCandidates[i].heightAboveRoad;
	      pngPath = pngfilename.str();
	      cv::putText(imgPressentation, pngPath, cv::Point(CandidateEvaluator.trafficSignalCandidates[i].rect2d.x,CandidateEvaluator.trafficSignalCandidates[i].rect2d.y), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar( 0, 55, 255 ), 2,3);
	      //cout << "Succeded! combined confidence: " << combinedConfidence << endl;
	    }
	    else
	    {
	    	circle(imgPressentation, cv::Point(CandidateEvaluator.trafficSignalCandidates[i].trafficSignalPosition2D.x, CandidateEvaluator.trafficSignalCandidates[i].trafficSignalPosition2D.y), 18, cv::Scalar( 200, 155, 200 ), 2, 8, 0);
	    	//cout << "Failed! combined confidence: " << combinedConfidence << endl;
	    }
      
    }
/*
    // Draw the circles detected
    for( size_t i = 0; i < EdgeSegmentation.circles.size(); i++ )
    {
        cv::Point center(cvRound(EdgeSegmentation.circles[i][0]), cvRound(EdgeSegmentation.circles[i][1]));
        int radius = cvRound(EdgeSegmentation.circles[i][2]);
        // circle center
        cv::circle( imgPressentation, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        cv::circle( imgPressentation, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
     }
*/
    cv::Mat combinedOutput=cv::Mat::zeros(IMAGEHEIGHT,IMAGEWIDTH*2+255,imgLOI.type());
    imgPressentation.copyTo(combinedOutput(cv::Rect(0,0,IMAGEWIDTH,IMAGEHEIGHT)));
    Pressentation.representGrayInColor(Disparity.obstacleImage, 0, 255).copyTo(combinedOutput(cv::Rect(IMAGEWIDTH,0,IMAGEWIDTH,IMAGEHEIGHT)));
    Pressentation.representGrayInColor(vdispout, 0, 255).copyTo(combinedOutput(cv::Rect(IMAGEWIDTH*2,0,255,IMAGEHEIGHT)));
    output.write(combinedOutput);
    cv::resize(combinedOutput,combinedOutput,cv::Size(IMAGEWIDTH+128,IMAGEHEIGHT/2),CV_INTER_LINEAR);
    imshow("combinedOutput",combinedOutput);
    
    
    int k = cv::waitKey(10);
    if (k=='q') {
      //imwrite("imgLOI.png", imgLOI); 
      output.release();
      break;
    }
    if(k=='s') {
      imwrite("imgLOI.png", imgLOI);
      imwrite("dispOutFinished.png", Pressentation.representGrayInColor(dispOutFinished, 0, 255));
      //PointCloud.createCompletePointCloud(imgLOI);
    }
    if(k=='p') {
      cv::waitKey();
    }
  }
  return 0;
}
