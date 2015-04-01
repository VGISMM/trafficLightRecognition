#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <iostream>
#include <string>
#include <stdio.h>

#include "../defines.h"
#import "../pointCloud/pointCloud.h"
#import "../colorSegmentation/colorLight/colorLight.h"
#import "../colorSegmentation/spotLight/spotLight.h"
#include "../trafficSignal/trafficSignal.h"
//using namespace cv;
using namespace std;

class CandidateEvaluator {
public:
	CandidateEvaluator();
	void collectCandidates(cv::vector<SpotLight> spotLights, cv::vector<ColorLight> colorLights);
	void stereoVisionEvaluation(PointCloud PointCloud);
	cv::vector<TrafficSignal> frameTrafficSignals;
	cv::vector<TrafficSignal> trafficSignalCandidates;
private:
	void detectorConfidencEvaluation();
};