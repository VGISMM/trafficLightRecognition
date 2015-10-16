#include <iostream>
#include <string>
#include <limits>
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "../trafficSignal/trafficSignal.h"
//#include "../defines.h"
#include <vector>

using namespace cv::ml;
using namespace std;

class Evaluation{
public:
	Evaluation();
	void init(string type);
	void loadGT();
	void endInputStream();
	void endOutputStream();
	void readFrameAnnotations(int frameNumber);
	void evaluateStandard(string frameName);
	void evaluateVIVA();
	void cleanUpCandidates();
	std::vector<cv::Rect> GTrects;
	ifstream *GTInputStream;
	cv::Mat annotationImage;
	std::vector<string> compressedAnnotationVector, expandedAnnotationVector; 
	std::vector<TrafficSignal> preTrafficSignalCandidates, trafficSignalCandidates;
private:
	//std::vector<string> &split(const string, char, std::vector<string>);
	
	std::vector<string> split(const string &s, char delim);

	ofstream scoreOutputStream, labelOutputStream;
	int frameNumberIndex;
	string scoresPath, labelPath, typeLabel;
	stringstream evalFileScoreStream, evalFileLabelStream;

	//string scoresPath,
	stringstream resultsOutputStreamString;
	string resultsPath;
	ofstream resultsOutputStreamFile;
    
	
	
int FPtotal = 0;
int TPtotal = 0;
int FNtotal = 0;
int FPframe = 0;
int TPframe = 0;
int FNframe = 0;


};
