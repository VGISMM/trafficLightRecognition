#include "backproject.h"

Backproject::Backproject(){} 

// Creating color histograms for backprojection
void Backproject::init(){
  const int channels[] = {1, 2};
  float range[] = { 0, 256 };
  const float *ranges[] = { range, range };
  const int histSize[] = { 256,256 };

  char path[200];
  cv::Mat greenTraining = cv::Mat::zeros(20,20*numberOfTrainingSamples,CV_8UC3);
  cv::Mat redTraining = cv::Mat::zeros(20,20*numberOfTrainingSamples,CV_8UC3);
  for(int trainingImageIndex=0;trainingImageIndex<numberOfTrainingSamples;trainingImageIndex++)
  {
    sprintf(path, "data/greenTraining/%i.png", trainingImageIndex);
    cv::imread(path,1).copyTo(greenTraining(cv::Rect(20*trainingImageIndex,0,20,20)));
  }

  for(int trainingImageIndex=0;trainingImageIndex<numberOfTrainingSamples;trainingImageIndex++)
  {
    sprintf(path, "data/redTraining/%i.png", trainingImageIndex);
    cv::imread(path,1).copyTo(redTraining(cv::Rect(20*trainingImageIndex,0,20,20)));
  }

  cvtColor(greenTraining, greenTraining, CV_BGR2Luv);
  calcHist(&greenTraining, 1, channels, cv::noArray(), greenHistogram, 2, histSize, ranges, true, false);
  normalize(greenHistogram, greenHistogram, 0, 255, cv::NORM_MINMAX); 

  cvtColor(redTraining, redTraining, CV_BGR2Luv);
  calcHist(&redTraining, 1, channels, cv::noArray(), redHistogram, 2, histSize, ranges, true, false);
  normalize(redHistogram, redHistogram, 0, 255, cv::NORM_MINMAX); 
  //imshow( "redTraining", redTraining );

  // Visualize histograms
  int w = 256; int h = 256;
  int bin_w = cvRound( (double) w / 256 );
  cv::Mat greenHistImg = cv::Mat::zeros( w, h, CV_8UC3 );
  cv::reduce(greenHistogram, objectHistogram1, 0, CV_REDUCE_SUM, -1);
  normalize(objectHistogram1, objectHistogram1, 0, 255, cv::NORM_MINMAX); 
  cv::reduce(greenHistogram, objectHistogram2, 1, CV_REDUCE_SUM, -1);
  normalize(objectHistogram2, objectHistogram2, 0, 255, cv::NORM_MINMAX); 
  for( int i = 1; i < 256; i++ )
  {
    line( greenHistImg, cv::Point( bin_w*(i-1), h - cvRound(objectHistogram1.at<float>(i-1)) ),
                        cv::Point( bin_w*(i), h - cvRound(objectHistogram1.at<float>(i)) ),
                        cv::Scalar( 0, 255, 0), 1, 8, 0  );
    line( greenHistImg, cv::Point( bin_w*(i-1), h - cvRound(objectHistogram2.at<float>(i-1)) ),
                        cv::Point( bin_w*(i), h - cvRound(objectHistogram2.at<float>(i)) ),
                        cv::Scalar( 255, 0, 0), 1, 8, 0  );
  }
  imwrite("data/greenTraining/greenHistImg.png", greenHistImg);

  cv::Mat redHistImg = cv::Mat::zeros( w, h, CV_8UC3 );
  cv::reduce(redHistogram, objectHistogram1, 0, CV_REDUCE_SUM, -1);
  normalize(objectHistogram1, objectHistogram1, 0, 255, cv::NORM_MINMAX); 
  cv::reduce(redHistogram, objectHistogram2, 1, CV_REDUCE_SUM, -1);
  normalize(objectHistogram2, objectHistogram2, 0, 255, cv::NORM_MINMAX); 
  for( int i = 1; i < 256; i++ )
  {
    line( redHistImg, cv::Point( bin_w*(i-1), h - cvRound(objectHistogram1.at<float>(i-1)) ),
                      cv::Point( bin_w*(i), h - cvRound(objectHistogram1.at<float>(i)) ),
                      cv::Scalar( 0, 255, 0), 1, 8, 0  );
    line( redHistImg, cv::Point( bin_w*(i-1), h - cvRound(objectHistogram2.at<float>(i-1)) ),
                      cv::Point( bin_w*(i), h - cvRound(objectHistogram2.at<float>(i)) ),
                      cv::Scalar( 255, 0, 0), 1, 8, 0  );
  }
  imwrite("data/redTraining/redHistImg.png", redHistImg);
}

void Backproject::backproject(cv::Mat frame){
  const int channels[] = {1, 2};
  float range[] = { 0, 256 };
  const float *ranges[] = { range, range };

  //imshow("frame",frame);
  calcBackProject(&frame, 1, channels, greenHistogram, greenBP, ranges);
  calcBackProject(&frame, 1, channels, redHistogram, redBP, ranges);
  threshold( greenBP, greenBP, 15, 255, 0 );
  threshold( redBP, redBP, 25, 255, 0 );

  //imshow("greenBP",greenBP);
  // Visualize the segmented lights

  vector<cv::Mat> BPchannels;

  BPchannels.push_back(redBP);
  BPchannels.push_back(greenBP);
  BPchannels.push_back(redBP);

  merge(BPchannels,outBP);

  cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
  dilate(outBP, outBP, sel, cv::Point(-1, -1), 2, 1, 3);
  morphologyEx( outBP, outBP, cv::MORPH_CLOSE, sel, cv::Point(-1,-1), 1 );
  morphologyEx( outBP, outBP, cv::MORPH_OPEN, sel, cv::Point(-1,-1), 1 );
  cvtColor(outBP, outBP, CV_RGB2GRAY);
  //cv::resize(outBP,outBP,cv::Size(),0.5,0.5,CV_INTER_LINEAR);
  //imshow("BPchannels",outBP);
}
