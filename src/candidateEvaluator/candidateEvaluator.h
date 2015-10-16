#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/contrib/contrib.hpp"
#include <iostream>
#include <string>
#include <stdio.h>

#include "../defines.h"

#ifdef stereo
#import "../stereo/pointCloud/pointCloud.h"
#endif

#import "../detectors/colorLight/colorLight.h"
#import "../detectors/spotLight/spotLight.h"
#include "../trafficSignal/trafficSignal.h"
//using namespace cv;
using namespace std;

class CandidateEvaluator {
public:
	CandidateEvaluator();

/*
	std::vector<TrafficSignal> frameSpotlightTrafficSignals, frameBackprojectTrafficSignals, frameColorThresholdTrafficSignals, frameColorGMMTrafficSignals;
	void collectBackprojectCandidates(std::vector<ColorLight> colorLights);
	void collectColorThresholdCandidates(std::vector<ColorLight> colorLights);
	void collectColorGMMCandidates(std::vector<ColorLight> colorLights);
	void collectSpotlightCandidates(std::vector<ColorLight> spotLights);
*/
	std::vector<TrafficSignal> frameTLs;
	void collectTLCandidates(std::vector<ColorLight> colorLights);
	//void noStereoVisionEvaluation();
	//void stereoVisionEvaluation(PointCloud PointCloud);
	//std::vector<TrafficSignal> frameTrafficSignals;
	//std::vector<TrafficSignal> frameSpotlightTrafficSignalsStereoVerified;
	//std::vector<TrafficSignal> frameColorTrafficSignalsStereoVerified;
	//std::vector<TrafficSignal> trafficSignalCandidates;
private:
	void detectorConfidencEvaluation();
};