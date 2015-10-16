#include "candidateEvaluator.h"
CandidateEvaluator::CandidateEvaluator	() {

}

void CandidateEvaluator::collectTLCandidates(std::vector<ColorLight> colorLights)
{
  
  frameTLs.clear();
  for( int i = 0; i< colorLights.size(); i++ )
  {
      TrafficSignal tempTL;
      tempTL.lampRect = colorLights[i].ROI;
      tempTL.rect2d = colorLights[i].ROI;
      tempTL.rectRatio = colorLights[i].rectRatio;
      tempTL.solidity = colorLights[i].solidity;
      tempTL.floodRatio = colorLights[i].floodRatio;
      tempTL.meanIntensity = colorLights[i].meanIntensity;
      tempTL.colorConfidence = colorLights[i].colorConfidence;
      if(colorLights[i].color == 0)
      {
        tempTL.trafficLightColor = cv::Scalar(255,0,0);
      }
      else if(colorLights[i].color == 1)
      {
        tempTL.trafficLightColor = cv::Scalar(0,255,0);
      }
      else if(colorLights[i].color == 2)
      {
        tempTL.trafficLightColor = cv::Scalar(0,0,255);
      }
      tempTL.calculateColorLightConfidence();
      frameTLs.push_back(tempTL);
  }
}

/*
void CandidateEvaluator::collectSpotlightCandidates(std::vector<ColorLight> colorLights)
{
  cv::Rect fullRect;
  frameSpotlightTrafficSignals.clear();
  for( int i = 0; i< colorLights.size(); i++ )
  {
      float widthHeightMean = (colorLights[i].ROI.width+colorLights[i].ROI.height)/2;
      if(colorLights[i].color == 0) //red
      {
        fullRect = cv::Rect(colorLights[i].ROI.x-widthHeightMean/2, 
          colorLights[i].ROI.y-widthHeightMean/2, 
          widthHeightMean*2, widthHeightMean*3.5);
      }
      else if(colorLights[i].color == 1) //yellow
      {
        fullRect = cv::Rect(colorLights[i].ROI.x-widthHeightMean/2, 
          colorLights[i].ROI.y-widthHeightMean*1, 
          widthHeightMean*2, widthHeightMean*3.5);
      }
      else if(colorLights[i].color == 2) //green
      {
        fullRect = cv::Rect(colorLights[i].ROI.x-widthHeightMean/2, 
          colorLights[i].ROI.y-widthHeightMean*2, 
          widthHeightMean*2, widthHeightMean*3.5);
      }

      TrafficSignal tempSpotlightTrafficSignal;
      //tempSpotlightTrafficSignal.spotLight = true;
      tempSpotlightTrafficSignal.lampRect = colorLights[i].ROI;
      tempSpotlightTrafficSignal.rect2d = fullRect;
      tempSpotlightTrafficSignal.rectRatio = colorLights[i].rectRatio;
      tempSpotlightTrafficSignal.solidity = colorLights[i].solidity;
      tempSpotlightTrafficSignal.floodRatio = colorLights[i].floodRatio;
      tempSpotlightTrafficSignal.trafficLightColor = cv::Scalar(255,0,0);
      tempSpotlightTrafficSignal.calculateSpotLightConfidence();
      frameSpotlightTrafficSignals.push_back(tempSpotlightTrafficSignal);
  }
}

void CandidateEvaluator::collectBackprojectCandidates(std::vector<ColorLight> colorLights)
{
  cv::Rect fullRect;
  frameBackprojectTrafficSignals.clear();
  for( int i = 0; i< colorLights.size(); i++ )
  {
      float widthHeightMean = (colorLights[i].ROI.width+colorLights[i].ROI.height)/2;
      if(colorLights[i].color == 0) //red
      {
        fullRect = cv::Rect(colorLights[i].ROI.x-widthHeightMean/2, 
          colorLights[i].ROI.y-widthHeightMean/2, 
          widthHeightMean*2, widthHeightMean*3.5);
      }
      else if(colorLights[i].color == 1) //yellow
      {
        fullRect = cv::Rect(colorLights[i].ROI.x-widthHeightMean/2, 
          colorLights[i].ROI.y-widthHeightMean*1, 
          widthHeightMean*2, widthHeightMean*3.5);
      }
      else if(colorLights[i].color == 2) //green
      {
        fullRect = cv::Rect(colorLights[i].ROI.x-widthHeightMean/2, 
          colorLights[i].ROI.y-widthHeightMean*2, 
          widthHeightMean*2, widthHeightMean*3.5);
      }

      TrafficSignal tempColorTrafficSignal;
      //tempColorTrafficSignal.colorLight = true;
      tempColorTrafficSignal.color = colorLights[i].color;
      tempColorTrafficSignal.lampRect = colorLights[i].ROI;
      tempColorTrafficSignal.rect2d = fullRect;
      tempColorTrafficSignal.meanIntensity = colorLights[i].meanIntensity;
      tempColorTrafficSignal.colorConfidence = colorLights[i].confidence;
      tempColorTrafficSignal.rectRatio = colorLights[i].rectRatio;
      tempColorTrafficSignal.floodRatio = colorLights[i].floodRatio;
      tempColorTrafficSignal.trafficLightColor = colorLights[i].color;
      tempColorTrafficSignal.calculateColorLightConfidence();
      frameBackprojectTrafficSignals.push_back(tempColorTrafficSignal);
  }
}

void CandidateEvaluator::collectColorThresholdCandidates(std::vector<ColorLight> colorLights)
{
  cv::Rect fullRect;
  frameColorThresholdTrafficSignals.clear();
  for( int i = 0; i< colorLights.size(); i++ )
  {
      float widthHeightMean = (colorLights[i].ROI.width+colorLights[i].ROI.height)/2;
      if(colorLights[i].color == 0) //red
      {
        fullRect = cv::Rect(colorLights[i].ROI.x-widthHeightMean/2, 
          colorLights[i].ROI.y-widthHeightMean/2, 
          widthHeightMean*2, widthHeightMean*3.5);
      }
      else if(colorLights[i].color == 1) //yellow
      {
        fullRect = cv::Rect(colorLights[i].ROI.x-widthHeightMean/2, 
          colorLights[i].ROI.y-widthHeightMean*1, 
          widthHeightMean*2, widthHeightMean*3.5);
      }
      else if(colorLights[i].color == 2) //green
      {
        fullRect = cv::Rect(colorLights[i].ROI.x-widthHeightMean/2, 
          colorLights[i].ROI.y-widthHeightMean*2, 
          widthHeightMean*2, widthHeightMean*3.5);
      }

      TrafficSignal tempColorTrafficSignal;
      //tempColorTrafficSignal.colorLight = true;
      tempColorTrafficSignal.color = colorLights[i].color;
      tempColorTrafficSignal.lampRect = colorLights[i].ROI;
      tempColorTrafficSignal.rect2d = fullRect;
      tempColorTrafficSignal.meanIntensity = colorLights[i].meanIntensity;
      tempColorTrafficSignal.colorConfidence = colorLights[i].confidence;
      tempColorTrafficSignal.rectRatio = colorLights[i].rectRatio;
      tempColorTrafficSignal.floodRatio = colorLights[i].floodRatio;
      tempColorTrafficSignal.trafficLightColor = colorLights[i].color;
      tempColorTrafficSignal.calculateColorLightConfidence();
      frameColorThresholdTrafficSignals.push_back(tempColorTrafficSignal);
  }
}

void CandidateEvaluator::collectColorGMMCandidates(std::vector<ColorLight> colorLights)
{
  cv::Rect fullRect;
  frameColorGMMTrafficSignals.clear();
  for( int i = 0; i< colorLights.size(); i++ )
  {
      float widthHeightMean = (colorLights[i].ROI.width+colorLights[i].ROI.height)/2;
      if(colorLights[i].color == 0) //red
      {
        fullRect = cv::Rect(colorLights[i].ROI.x-widthHeightMean/2, 
          colorLights[i].ROI.y-widthHeightMean, 
          widthHeightMean*2, widthHeightMean*4.5);
      }
      else if(colorLights[i].color == 1) //yellow
      {
        fullRect = cv::Rect(colorLights[i].ROI.x-widthHeightMean/2, 
          colorLights[i].ROI.y-widthHeightMean*1.5, 
          widthHeightMean*2, widthHeightMean*4.5);
      }
      else if(colorLights[i].color == 2) //green
      {
        fullRect = cv::Rect(colorLights[i].ROI.x-widthHeightMean/2, 
          colorLights[i].ROI.y-widthHeightMean*3, 
          widthHeightMean*2, widthHeightMean*4.5);
      }

      TrafficSignal tempColorTrafficSignal;
      //tempColorTrafficSignal.colorLight = true;
      tempColorTrafficSignal.color = colorLights[i].color;
      tempColorTrafficSignal.lampRect = colorLights[i].ROI;
      tempColorTrafficSignal.rect2d = fullRect;
      tempColorTrafficSignal.meanIntensity = colorLights[i].meanIntensity;
      tempColorTrafficSignal.colorConfidence = colorLights[i].confidence;
      tempColorTrafficSignal.rectRatio = colorLights[i].rectRatio;
      tempColorTrafficSignal.floodRatio = colorLights[i].floodRatio;
      tempColorTrafficSignal.trafficLightColor = colorLights[i].color;
      tempColorTrafficSignal.calculateColorLightConfidence();
      frameColorGMMTrafficSignals.push_back(tempColorTrafficSignal);
  }
}
*/

/*
void CandidateEvaluator::noStereoVisionEvaluation(){ 
  frameSpotlightTrafficSignalsStereoVerified.clear();
  //cout << "11size " << frameSpotlightTrafficSignals.size() << endl;
  for( int i = 0; i<frameSpotlightTrafficSignals.size(); i++ )
  {
    frameSpotlightTrafficSignalsStereoVerified.push_back(frameSpotlightTrafficSignals[i]);
  }
  //cout << "her !! 2" << endl;
  frameColorTrafficSignalsStereoVerified.clear();
  for( int i = 0; i<frameColorTrafficSignals.size(); i++ )
  {
    frameColorTrafficSignalsStereoVerified.push_back(frameColorTrafficSignals[i]);
  }
  //detectorConfidencEvaluation();
}

void CandidateEvaluator::stereoVisionEvaluation(PointCloud PointCloud){ 
  //cout << "her !! 1" << endl;
  frameSpotlightTrafficSignalsStereoVerified.clear();
  for( int i = 0; i<frameSpotlightTrafficSignals.size(); i++ )
  {
    //cout << "height: " << PointCloud.pointDistanceFromPlane << endl;
    if (PointCloud.projectRegionToPointCloud(frameSpotlightTrafficSignals[i].lampRect))
    {
      //
      if (PointCloud.pointDistanceFromPlane > 2 && PointCloud.pointDistanceFromPlane < 8.0)
      {
        //cout << "height: " << PointCloud.pointDistanceFromPlane << endl;
        frameSpotlightTrafficSignals[i].trafficSignalPosition = PointCloud.clusterFront3Dpoint;
        frameSpotlightTrafficSignals[i].heightAboveRoad = PointCloud.pointDistanceFromPlane; 
        frameSpotlightTrafficSignals[i].trafficSignalPlanePoint = PointCloud.pointProjected2Plane;
        frameSpotlightTrafficSignals[i].findTrafficSignalsImageCoordiantes();
        frameSpotlightTrafficSignalsStereoVerified.push_back(frameSpotlightTrafficSignals[i]);
      }
    }
  }
  //cout << "her !! 2" << endl;
  frameColorTrafficSignalsStereoVerified.clear();
  for( int i = 0; i<frameColorTrafficSignals.size(); i++ )
  {
    if (PointCloud.projectRegionToPointCloud(frameColorTrafficSignals[i].lampRect))
    {

      if (PointCloud.pointDistanceFromPlane > 2 && PointCloud.pointDistanceFromPlane < 8.0)
      {
        frameColorTrafficSignals[i].trafficSignalPosition = PointCloud.clusterFront3Dpoint;
        frameColorTrafficSignals[i].heightAboveRoad = PointCloud.pointDistanceFromPlane; 
        frameColorTrafficSignals[i].trafficSignalPlanePoint = PointCloud.pointProjected2Plane;
        frameColorTrafficSignals[i].findTrafficSignalsImageCoordiantes();
        frameColorTrafficSignalsStereoVerified.push_back(frameColorTrafficSignals[i]);
      }
    }
  }
  //detectorConfidencEvaluation();
}
*/
