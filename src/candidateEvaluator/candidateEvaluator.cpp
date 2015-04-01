#include "candidateEvaluator.h"
CandidateEvaluator::CandidateEvaluator	() {

}

void CandidateEvaluator::collectCandidates(cv::vector<SpotLight> spotLights, cv::vector<ColorLight> colorLights){ 
	frameTrafficSignals.clear();
	bool intersect = false;
  int winningIndex;

  // Spot light candidate evaluation
  for( int i = 0; i< spotLights.size(); i++ )
  {
    for( int j = 0; j< frameTrafficSignals.size(); j++ )
    {
      cv::Point2f diff;
      diff.x = spotLights[i].ROI.x+spotLights[i].ROI.width/2 - frameTrafficSignals[j].rect2d.x+frameTrafficSignals[j].rect2d.width/2;
      diff.y = spotLights[i].ROI.y+spotLights[i].ROI.height/2 - frameTrafficSignals[j].rect2d.y+frameTrafficSignals[j].rect2d.height/2;
      float eucliadianDistance = cv::sqrt(diff.x*diff.x + diff.y*diff.y);
      
      if(abs(eucliadianDistance) < eucliadianDistanceThreshold)
      {
        intersect = true;
        winningIndex = j;
      }
    }
    if(intersect)
    {
      intersect = false;
      frameTrafficSignals[winningIndex].spotLight = true;
      frameTrafficSignals[winningIndex].rect2d = frameTrafficSignals[winningIndex].rect2d | spotLights[i].ROI; // should not be used
      frameTrafficSignals[winningIndex].spotLightRectRatio = spotLights[i].rectRatio;
      frameTrafficSignals[winningIndex].spotLightSolidity = spotLights[i].solidity;
      frameTrafficSignals[winningIndex].spotLightFloodRatio = spotLights[i].floodRatio;
      if(frameTrafficSignals[winningIndex].colorLight)
      {
        frameTrafficSignals[winningIndex].trafficLightColor = cv::Scalar(255,0,255);
      }
      else
      {
        frameTrafficSignals[winningIndex].trafficLightColor = cv::Scalar(255,55,0);
      }
      frameTrafficSignals[winningIndex].calculateSpotLightConfidence();
    }
    else
    {
      // Push to vector with all accepted TLs
      TrafficSignal tempTrafficSignal;
      tempTrafficSignal.spotLight = true;
      tempTrafficSignal.rect2d = spotLights[i].ROI; // should not be used
      tempTrafficSignal.spotLightRectRatio = spotLights[i].rectRatio;
      tempTrafficSignal.spotLightSolidity = spotLights[i].solidity;
      tempTrafficSignal.spotLightFloodRatio = spotLights[i].floodRatio;
      tempTrafficSignal.trafficLightColor = cv::Scalar(255,0,0);
      tempTrafficSignal.calculateSpotLightConfidence();
      frameTrafficSignals.push_back(tempTrafficSignal);
    }
  }

  // Color light candidate evaluation
  for( int i = 0; i< colorLights.size(); i++ )
  {
    for( int j = 0; j< frameTrafficSignals.size(); j++ )
    {
      cv::Point2f diff;
      diff.x = colorLights[i].ROI.x+colorLights[i].ROI.width/2 - frameTrafficSignals[j].rect2d.x+frameTrafficSignals[j].rect2d.width/2;
      diff.y = colorLights[i].ROI.y+colorLights[i].ROI.height/2 - frameTrafficSignals[j].rect2d.y+frameTrafficSignals[j].rect2d.height/2;
      float eucliadianDistance = cv::sqrt(diff.x*diff.x + diff.y*diff.y);
      
      if(abs(eucliadianDistance) < eucliadianDistanceThreshold)
      {
        intersect = true;
        winningIndex = j;
      }
    }
    if(intersect)
    {
      intersect = false;
      frameTrafficSignals[winningIndex].colorLight = true;
      frameTrafficSignals[winningIndex].rect2d = frameTrafficSignals[winningIndex].rect2d | colorLights[i].ROI; // should not be used
      frameTrafficSignals[winningIndex].colorConfidence = colorLights[i].confidence;
      frameTrafficSignals[winningIndex].colorRectRatio = colorLights[i].rectRatio;
      frameTrafficSignals[winningIndex].colorFloodRatio = colorLights[i].floodRatio;
      if(frameTrafficSignals[winningIndex].spotLight)
      {
        frameTrafficSignals[winningIndex].trafficLightColor = cv::Scalar(255,0,255);
      }
      else
      {
        frameTrafficSignals[winningIndex].trafficLightColor = cv::Scalar(0,55,255);
      }
      frameTrafficSignals[winningIndex].calculateColorLightConfidence();
    }
    else
    {
      // Push to vector with all accepted TLs
      TrafficSignal tempTrafficSignal;
      tempTrafficSignal.colorLight = true;
      tempTrafficSignal.rect2d = colorLights[i].ROI; // should not be used
      tempTrafficSignal.colorConfidence = colorLights[i].confidence;
      tempTrafficSignal.colorRectRatio = colorLights[i].rectRatio;
      tempTrafficSignal.colorFloodRatio = colorLights[i].floodRatio;
      tempTrafficSignal.trafficLightColor = cv::Scalar(0,0,255);
      tempTrafficSignal.calculateColorLightConfidence();
      frameTrafficSignals.push_back(tempTrafficSignal);
    }
  }

  // Shape light candidate evaluation  
}

void CandidateEvaluator::stereoVisionEvaluation(PointCloud PointCloud){ 
  trafficSignalCandidates.clear();
  for( int i = 0; i<frameTrafficSignals.size(); i++ )
  {
    if (PointCloud.projectRegionToPointCloud(frameTrafficSignals[i].rect2d))
    {

      if (PointCloud.pointDistanceFromPlane > 3.0 && PointCloud.pointDistanceFromPlane < 9.0)
      {
        frameTrafficSignals[i].trafficSignalPosition = PointCloud.clusterFront3Dpoint;
        frameTrafficSignals[i].heightAboveRoad = PointCloud.pointDistanceFromPlane; 
        frameTrafficSignals[i].trafficSignalPlanePoint = PointCloud.pointProjected2Plane;
        frameTrafficSignals[i].findTrafficSignalsImageCoordiantes();
        trafficSignalCandidates.push_back(frameTrafficSignals[i]);
      }
    }
  }
  //detectorConfidencEvaluation();
}

/*
void CandidateEvaluator::detectorConfidencEvaluation(){ 
  
  for( int i = 0; i<trafficSignalCandidates.size(); i++ )
  {
    if(trafficSignalCandidates[i].colorLight)
    {

    }

    if(trafficSignalCandidates[i].spotLight)
    {

    }

  }
}
*/
