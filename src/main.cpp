#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <thread>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//#include "blobAnalysis/blobAnalysis.h"
#include "pressentation/pressentation.h"
#include "candidateEvaluator/candidateEvaluator.h"
#include "evaluation/evaluation.h"
#include "detectors/backproject.h"
//#include "detectors/GMM.h"
#include "detectors/thresholdColor.h"
#include "detectors/spotLightDetection.h"
#include "detectors/edgeSegmentation.h"


#include "tracking/kalmanfilter2d.h"
#include "trafficSignal/trafficSignal.h"

#ifdef stereo
#include "stereo/ransac/ransac.h"
#include "stereo/disparity/disparity.h"
#include "stereo/pointCloud/pointCloud.h"
#endif

using namespace std;

cv::VideoWriter output;

std::vector<TrafficSignal> spotlightTrafficSignals;
std::vector<TrafficSignal> colorThresholdTrafficSignals;
std::vector<TrafficSignal> backprojectTrafficSignals;
std::vector<TrafficSignal> colorGMMTrafficSignals;

int frameNumber = 0;
std::thread t_spotlight, t_backproject, t_colorThreshold, t_colorGMM;

cv::Mat img, imgBGR, imgROI;  // Input images  , imgLGray, imgRGray; 
cv::Mat dstYCrCb, dstLUV; // Colorspaces
cv::Mat colorChannels[3];

#ifdef stereo
  bool first = true;
  float thresholdOffset = -5.1;
  cv::Mat dispOutFinished, dispLower, dispUpper, vdispout; // stereo images
#endif

std::string ZeroPadNumber(int num)
{
    std::ostringstream ss;
    ss << std::setw( 5 ) << std::setfill( '0' ) << num;
    return ss.str();
}

  string getFileExt(const string& s) {

   size_t i = s.rfind('.', s.length());
   if (i != string::npos) {
      return(s.substr(i+1, s.length() - i));
   }

   return("");
}

string getFileName(const string& s) {

   char sep = '/';

   size_t i = s.rfind(sep, s.length());
   if (i != string::npos) {
      return(s.substr(i+1, s.length() - i));
   }

   return("");
}

string replaceInName(const string& s, string key, string replacement) {

  std::string str = s;

  std::size_t found = s.rfind(key);
  if (found!=std::string::npos)
  {
    return(str.replace (found,key.length(),replacement));
  }

   return("");
}



void detectSpotlights(SpotLightDetection* SpotLightDetection, cv::Mat intensity, cv::Mat BGR) 
{
  SpotLightDetection->spotLightBlobAnalysis.detectedTLs.clear();
  SpotLightDetection->colorFrame = BGR;
  SpotLightDetection->intensityChannel = intensity;
  SpotLightDetection->segmentSpotLights();
  //cvtColor(SpotLightDetection->topHat, spotlightOut, CV_GRAY2BGR);

  CandidateEvaluator CandidateEvaluator;
  CandidateEvaluator.collectTLCandidates(SpotLightDetection->spotLightBlobAnalysis.detectedTLs);
  //cout << "spot size " << CandidateEvaluator.frameSpotlightTrafficSignals.size() << endl;
  spotlightTrafficSignals = CandidateEvaluator.frameTLs;
}

void detectBackprojects(Backproject* Backproject, cv::Mat intensity, cv::Mat BGR) 
{
  Backproject->backprojBlobAnalysis.detectedTLs.clear();
  Backproject->colorFrame = BGR;
  Backproject->intensity = intensity;
  Backproject->detectColors();

  CandidateEvaluator CandidateEvaluator;
  CandidateEvaluator.collectTLCandidates(Backproject->backprojBlobAnalysis.detectedTLs);
  backprojectTrafficSignals = CandidateEvaluator.frameTLs;
  //cout << "backproj size " << backprojectTrafficSignals.size() << endl;
}

void detectColorThresholds(ThresholdColor* ThresholdColor, cv::Mat intensity, cv::Mat BGR) 
{
  
  ThresholdColor->colorThreshBlobAnalysis.detectedTLs.clear();
  ThresholdColor->colorFrame = BGR;
  ThresholdColor->intensity = intensity;
  ThresholdColor->detectColors();

  CandidateEvaluator CandidateEvaluator;
  CandidateEvaluator.collectTLCandidates(ThresholdColor->colorThreshBlobAnalysis.detectedTLs);
  colorThresholdTrafficSignals = CandidateEvaluator.frameTLs;
  //cout << "backproj size " << backprojectTrafficSignals.size() << endl;
}

/*
void detectColorGMMs(GMM* GMM, cv::Mat intensity, cv::Mat BGR) 
{
  
  GMM->detectedColorLights.clear();
  GMM->colorFrame = BGR;
  GMM->intensity = intensity;
  GMM->detectColors();

  CandidateEvaluator CandidateEvaluator;
  CandidateEvaluator.collectTLCandidates(GMM->detectedColorLights);
  colorGMMTrafficSignals = CandidateEvaluator.frameTLs;
  //cout << "backproj size " << backprojectTrafficSignals.size() << endl;
}
*/
int main(int argc, char **argv) {

#ifdef stereo
  Disparity Disparity;
  PointCloud PointCloud;
  Ransac Ransac;
  Kalman Kalman;
#endif
  EdgeSegmentation EdgeSegmentation;
  SpotLightDetection SpotLightDetection;
  ThresholdColor ThresholdColor;
  Backproject Backproject;
  //GMM GMM;
  
  Evaluation EvaluationSpotlight;
  Evaluation EvaluationBackproject;
  Evaluation EvaluationGMM;
  Evaluation EvaluationThresholdColor;

  Pressentation Pressentation;

  std::string inputPath;
  std::string initialFileName, fileName;
  std::string pathToData(getFileExt(argv[1]));
  std::cout << "The extension is \"" << pathToData << "\"\n";

  if(pathToData == "avi" || pathToData == "mp4")
  {
    initialFileName = "00000.png";
    inputPath = argv[1];
  }
  else if(pathToData == "png")
  {
    initialFileName = getFileName(argv[1]);
    inputPath = replaceInName(argv[1], "00000", "%05d");
    std::cout << "The file name is \"" << initialFileName << "\"\n";
  }
  else
  {
    initialFileName = "00000.png";
    inputPath = argv[1];
    std::cout << "Input file is unsupported" << endl;
    
  }
  cv::VideoCapture capture(inputPath);

#ifdef stereo
  output.open( "../Data/output/out.avi", CV_FOURCC('M','J','P','G'), 16, cv::Size (IMAGEWIDTH*4+255,IMAGEHEIGHT*1.5), true );
#else
  output.open( "../Data/output/out.avi", CV_FOURCC('M','J','P','G'), 16, cv::Size (IMAGEWIDTH*3,IMAGEHEIGHT*1.5), true );
#endif

  EvaluationSpotlight.init("spot");
  EvaluationBackproject.init("back");
  EvaluationThresholdColor.init("thre");
  //EvaluationGMM.init("gmm");

  std::cout << "path to annotations: " << argv[2] << std::endl;
  ifstream inputStream(argv[2]);
  EvaluationBackproject.GTInputStream = &inputStream;
  EvaluationBackproject.loadGT();

  EvaluationThresholdColor.compressedAnnotationVector = EvaluationBackproject.compressedAnnotationVector;
  EvaluationThresholdColor.expandedAnnotationVector = EvaluationBackproject.expandedAnnotationVector;

  //EvaluationGMM.compressedAnnotationVector = EvaluationBackproject.compressedAnnotationVector;
  //EvaluationGMM.expandedAnnotationVector = EvaluationBackproject.expandedAnnotationVector;

  EvaluationSpotlight.compressedAnnotationVector = EvaluationBackproject.compressedAnnotationVector;
  EvaluationSpotlight.expandedAnnotationVector = EvaluationBackproject.expandedAnnotationVector;

  Backproject.init();
  //GMM.init();

  while(1) {
//-----------------------------------Input handling-------------------------------------------------
    // Get left and right stereo image pair 
    capture >> img;
    if (img.data==NULL){
      output.release();
      break;
    }
    imgBGR = img(cv::Rect(cv::Point(0,0), cv::Size(IMAGEWIDTH, IMAGEHEIGHT/2)));
    //imwrite("imgBGR.png",imgBGR);

    //imgBGR = img(cv::Rect(cv::Point(0,0), cv::Size(IMAGEWIDTH, IMAGEHEIGHT/2))).clone();
    //imgPressentationSpotlight = img(cv::Rect(cv::Point(0,0), cv::Size(IMAGEWIDTH, IMAGEHEIGHT/2))).clone();
    //imgROI = img(cv::Rect(cv::Point(IMAGEWIDTH,0), cv::Size(IMAGEWIDTH, IMAGEHEIGHT)));
//-----------------------------------Colorspace conversion-------------------------------------------
    // color space tests    
    //vector<cv::Mat> customChannels;
    //cv::Mat channelYCrCb[3];
    //cvtColor(imgBGR, dstYCrCb, CV_BGR2YCrCb); //YCrCb JPEG (or YCC)
    //split(dstYCrCb, channelYCrCb);
    cvtColor(imgBGR, dstLUV, CV_BGR2Luv);
    split(dstLUV, colorChannels);
    //customChannels.push_back(channelLuv[0]);
    //customChannels.push_back(channelLuv[1]);
    //customChannels.push_back(channelLuv[2]);
    //merge(customChannels,dstLUV); // combined
//-----------------------------------Disp calculations-----------------------------------------------
#ifdef stereo
    imgLOI = img(cv::Rect(cv::Point(0,0), cv::Size(IMAGEWIDTH, IMAGEHEIGHT)));
    imgROI = img(cv::Rect(cv::Point(IMAGEWIDTH,0), cv::Size(IMAGEWIDTH, IMAGEHEIGHT)));
    Disparity.imgBGRGPU.upload(imgLOI);
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
    // GPU vdisp
    Disparity.generateVdisp(dispOutFinished);
    vdispout = Disparity.vdisp.clone();
    Ransac.runRansac(vdispout);
    // Initilize Kalman at first frame
    if (first)
    {
      Kalman.initKalman((float)Ransac.slope,(float)Ransac.intersection);
      first = false;
    }
    Kalman.kalmanCorrect((float)Ransac.slope, (float)Ransac.intersection);
    Kalman.kalmanPredict();
    // Show the found road surface on the vdisp 
    circle(vdispout, Ransac.ransacPoint1, 3, cvScalar(150,150,150), 2, 8, 0);
    circle(vdispout, Ransac.ransacPoint2, 3, cvScalar(220,220,220), 2, 8, 0);
    line(vdispout, cv::Point((double)20,(double)(Ransac.slope*20+Ransac.intersection)-thresholdOffset),  cv::Point((double)80,(double)(Ransac.slope*80+Ransac.intersection)-thresholdOffset), cvScalar(155,155,155), 1, CV_AA);
    line(vdispout, cv::Point((double)20,(double)(Kalman.estimated.at<float>(0)*20+Kalman.estimated.at<float>(1))-thresholdOffset),  cv::Point((double)80,(double)(Kalman.estimated.at<float>(0)*80+Kalman.estimated.at<float>(1))-thresholdOffset), cvScalar(255,255,255), 1, CV_AA);
    // Threshold the disparity image just below the found road surface
    Disparity.vDispThresholdedImage(dispOutFinished, Kalman.estimated.at<float>(0), Kalman.estimated.at<float>(1), thresholdOffset);
    // Initiallize the pointcloud obejct with depth and color info
    PointCloud.init(Disparity.obstacleImage, imgLOI.clone());
    PointCloud.findRoadSurfaceCoefficients();
#endif  
//-----------------------------------Traffic light Segmentation-----------------------------------------------
    t_spotlight = std::thread(detectSpotlights, &SpotLightDetection, colorChannels[0].clone(), imgBGR.clone());
    
    t_colorThreshold = std::thread(detectColorThresholds, &ThresholdColor, colorChannels[0].clone(), imgBGR.clone());

    t_backproject = std::thread(detectBackprojects, &Backproject, colorChannels[0].clone(), imgBGR.clone());
    
    //t_colorGMM = std::thread(detectColorGMMs, &GMM, colorChannels[0].clone(), imgBGR.clone());
    
/*
#ifdef stereo
    CandidateEvaluator.stereoVisionEvaluation(PointCloud);
#else
    CandidateEvaluator.noStereoVisionEvaluation();
#endif
*/
    t_spotlight.join();
    t_colorThreshold.join();
    t_backproject.join();
    //t_colorGMM.join();
 //-----------------------------------GT stuff-----------------------------------------------



    fileName = replaceInName(initialFileName, "00000", ZeroPadNumber(frameNumber)); 
    EvaluationSpotlight.annotationImage = imgBGR.clone();
    EvaluationSpotlight.preTrafficSignalCandidates = spotlightTrafficSignals;
    EvaluationSpotlight.readFrameAnnotations(frameNumber);
    EvaluationSpotlight.evaluateStandard(fileName);

    EvaluationThresholdColor.annotationImage = imgBGR.clone();
    EvaluationThresholdColor.preTrafficSignalCandidates = colorThresholdTrafficSignals;
    EvaluationThresholdColor.readFrameAnnotations(frameNumber);
    EvaluationThresholdColor.evaluateStandard(fileName);

    EvaluationBackproject.annotationImage = imgBGR.clone();
    EvaluationBackproject.preTrafficSignalCandidates = backprojectTrafficSignals;
    EvaluationBackproject.readFrameAnnotations(frameNumber);
    EvaluationBackproject.evaluateStandard(fileName);

    /*
    EvaluationGMM.annotationImage = imgBGR.clone();
    EvaluationGMM.trafficSignalCandidates = colorGMMTrafficSignals;
    EvaluationGMM.readFrameAnnotations(frameNumber);
    EvaluationGMM.evaluate();
	  */
	
    stringstream frameNumberStream;
    frameNumberStream << frameNumber;
    string frameNumberString = frameNumberStream.str();
    putText(EvaluationBackproject.annotationImage, frameNumberString, cv::Point(40,40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(255,255,0), 3,3);
  
#ifdef stereo
    cv::Mat combinedOutput=cv::Mat::zeros(IMAGEHEIGHT*1.5,IMAGEWIDTH*2+255,imgBGR.type());
#else
    cv::Mat combinedOutput=cv::Mat::zeros(IMAGEHEIGHT*1.5,IMAGEWIDTH*3,imgBGR.type());
#endif

    EvaluationBackproject.annotationImage.copyTo(combinedOutput(cv::Rect(0,0,IMAGEWIDTH,IMAGEHEIGHT/2)));
    EvaluationThresholdColor.annotationImage.copyTo(combinedOutput(cv::Rect(IMAGEWIDTH,0,IMAGEWIDTH,IMAGEHEIGHT/2)));
    //EvaluationGMM.annotationImage.copyTo(combinedOutput(cv::Rect(IMAGEWIDTH*2,0,IMAGEWIDTH,IMAGEHEIGHT/2)));
    EvaluationSpotlight.annotationImage.copyTo(combinedOutput(cv::Rect(IMAGEWIDTH*2,0,IMAGEWIDTH,IMAGEHEIGHT/2)));

    Backproject.backprojBlobAnalysis.presentationImage.copyTo(combinedOutput(cv::Rect(0,IMAGEHEIGHT/2,IMAGEWIDTH,IMAGEHEIGHT/2)));
    ThresholdColor.colorThreshBlobAnalysis.presentationImage.copyTo(combinedOutput(cv::Rect(IMAGEWIDTH,IMAGEHEIGHT/2,IMAGEWIDTH,IMAGEHEIGHT/2)));
    //GMM.GMMoutput.copyTo(combinedOutput(cv::Rect(IMAGEWIDTH*2,IMAGEHEIGHT/2,IMAGEWIDTH,IMAGEHEIGHT/2)));
    SpotLightDetection.spotLightBlobAnalysis.presentationImage.copyTo(combinedOutput(cv::Rect(IMAGEWIDTH*2,IMAGEHEIGHT/2,IMAGEWIDTH,IMAGEHEIGHT/2)));

    Backproject.outBP.copyTo(combinedOutput(cv::Rect(0,IMAGEHEIGHT,IMAGEWIDTH,IMAGEHEIGHT/2)));
    ThresholdColor.outputTC.copyTo(combinedOutput(cv::Rect(IMAGEWIDTH,IMAGEHEIGHT,IMAGEWIDTH,IMAGEHEIGHT/2)));
    //GMM.GMMoutput.copyTo(combinedOutput(cv::Rect(IMAGEWIDTH*2,IMAGEHEIGHT/2,IMAGEWIDTH,IMAGEHEIGHT/2)));
    cv::Mat tophatBGR;
    cvtColor(SpotLightDetection.topHat, tophatBGR, CV_GRAY2BGR);
    tophatBGR.copyTo(combinedOutput(cv::Rect(IMAGEWIDTH*2,IMAGEHEIGHT,IMAGEWIDTH,IMAGEHEIGHT/2)));


#ifdef stereo
    Pressentation.representGrayInColor(dispUpper, 0, 255).copyTo(combinedOutput(cv::Rect(0,IMAGEHEIGHT,IMAGEWIDTH,IMAGEHEIGHT/2)));
    Pressentation.representGrayInColor(dispLower, 0, 255).copyTo(combinedOutput(cv::Rect(IMAGEWIDTH,IMAGEHEIGHT,IMAGEWIDTH,IMAGEHEIGHT/2)));
    Pressentation.representGrayInColor(vdispout, 0, 255).copyTo(combinedOutput(cv::Rect(IMAGEWIDTH*2,IMAGEHEIGHT/2,255,IMAGEHEIGHT)));
#endif    

    output.write(combinedOutput);

#ifdef IMSHOWON
#ifdef stereo
    cv::resize(combinedOutput,combinedOutput,cv::Size(IMAGEWIDTH+128,IMAGEHEIGHT*2/3),CV_INTER_LINEAR);
#else
    cv::resize(combinedOutput,combinedOutput,cv::Size(IMAGEWIDTH*1.5,IMAGEHEIGHT*1.5/2),CV_INTER_LINEAR);
#endif

    imshow("combinedOutput",combinedOutput);
    //imshow("mything",Backproject.mything);
#endif   
    //cv::waitKey();
    int k = cv::waitKey(10);
    if (k=='q') {
      //imwrite("imgBGR.png", imgBGR); 
      output.release();
      break;
    }
    if(k=='s') {
      imwrite("imgBGR.png", imgBGR);
      //PointCloud.createCompletePointCloud(imgBGR);
    }
    if(k=='p') {
      cv::waitKey();
    }
    frameNumber++;
  }

  EvaluationSpotlight.endOutputStream();
  EvaluationThresholdColor.endOutputStream();
  //EvaluationGMM.endOutputStream();

  EvaluationBackproject.endInputStream();
  EvaluationBackproject.endOutputStream(); 

  return 0;
}
