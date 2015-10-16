 //not working with opencv 3 because of legacy.h
#include "GMM.h"
GMM::GMM(){
	colorModel = EM::create();

    colorModel = StatModel::load<EM>("models/colorModel");

    gmmModelRed = EM::create();
    gmmModelYellow = EM::create();
    gmmModelGreen = EM::create();

    gmmModelRed = StatModel::load<EM>("models/gmmModelRed");
    gmmModelYellow = StatModel::load<EM>("models/gmmModelYellow");
 	gmmModelGreen = StatModel::load<EM>("models/gmmModelGreen");
}


void GMM::detectColors(){
  
  redGMM = cv::Mat::zeros( cv::Size( colorFrame.cols, colorFrame.rows ), CV_8UC1 );
  greenGMM = cv::Mat::zeros( cv::Size( colorFrame.cols, colorFrame.rows ), CV_8UC1 );
  yellowGMM = cv::Mat::zeros( cv::Size( colorFrame.cols, colorFrame.rows ), CV_8UC1 );

  predictFrame();
  
  threshold( redGMM, redGMM, 15, 255, 0 );
  threshold( yellowGMM, yellowGMM, 15, 255, 0 );
  threshold( greenGMM, greenGMM, 15, 255, 0 );

  locateTrafficLights(redGMM, 0);
  locateTrafficLights(yellowGMM, 1);
  locateTrafficLights(greenGMM, 2);

  // Visualize the segmented lights directly
  std::vector<cv::Mat> GMMchannels;
  GMMchannels.push_back(yellowGMM);
  GMMchannels.push_back(greenGMM);
  GMMchannels.push_back(redGMM);
  merge(GMMchannels, GMMoutput);
  /*
    for( int i = 0; i< detectedColorLights.size(); i++ )
  {
    rectangle(outBP, detectedColorLights[i].ROI, cv::Scalar( 0, 255, 255 ), 1, 4 );
  }
  cv::resize(outBP,outBP,cv::Size(),0.5,0.5,CV_INTER_LINEAR);
  imshow("TCChannels",outBP); 
  */
}

void GMM::predictFrame(){
    cvtColor(colorFrame, converted, CV_BGR2YCrCb);

    cv::Mat sample( 1, 3, CV_32FC1 );
    
    for( int i = 0; i < colorFrame.rows; i++ )
    {
        for(int j = 0; j < colorFrame.cols; j++ )
        {
            int Y = converted.at<cv::Vec3b>(i,j)[0];
            int Cr = converted.at<cv::Vec3b>(i,j)[1];
            int Cb = converted.at<cv::Vec3b>(i,j)[2];

            sample.at<float>(0) = (float)Y;
            sample.at<float>(1) = (float)Cr;
            sample.at<float>(2) = (float)Cb;

            float thresholdY = -10.1;
            float thresholdG = -10.1;
            float thresholdR = -0.1;
/*
            if(colorModel->predict2( sample, cv::noArray() )[0] < thresholdR){
                //cout << gmmModelRed->predict2( sample, cv::noArray() )[0] << endl;
                int response = cvRound(colorModel->predict2( sample, cv::noArray() )[1]);
                if (response==0){
                    //imgOut.at<cv::Vec3b>(i,j)[0] = 255;
                } //Trash clusters
                if (response==1){
                    imgOut.at<cv::Vec3b>(i,j)[1] = 255;
                } //Trash clusters
                if (response==2){
                    imgOut.at<cv::Vec3b>(i,j)[2] = 255;
                } //Trash clusters
            }
            */
            
            if(gmmModelYellow->predict2( sample, cv::noArray() )[0] < thresholdY){

                int response = cvRound(gmmModelYellow->predict2( sample, cv::noArray() )[1]);
                if (response==1){
                    yellowGMM.at<uchar>(i,j) = 255;
                } //Trash clusters

            }
            if(gmmModelGreen->predict2( sample, cv::noArray() )[0] < thresholdG){

                int response = cvRound(gmmModelGreen->predict2( sample, cv::noArray() )[1]);
                if (response==1){
                    greenGMM.at<uchar>(i,j) = 255;
                } //Trash clusters

            }
            if(gmmModelRed->predict2( sample, cv::noArray() )[0] < thresholdR){
                //cout << gmmModelRed->predict2( sample, cv::noArray() )[0] << endl;
                int response = cvRound(gmmModelRed->predict2( sample, cv::noArray() )[1]);
                if (response==1){
                    redGMM.at<uchar>(i,j) = 255;
                } //Trash clusters
            }
        }
    }
}

void GMM::locateTrafficLights(cv::Mat GMMChannel, int color){
  cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
  //dilate(TCChannel, TCChannel, sel, cv::Point(-1, -1), 2, 1, 3);
  morphologyEx( GMMChannel, GMMChannel, cv::MORPH_CLOSE, sel, cv::Point(-1,-1), 1 );
  morphologyEx( GMMChannel, GMMChannel, cv::MORPH_OPEN, sel, cv::Point(-1,-1), 1 );
  std::vector<std::vector<cv::Point>> contours; 
  std::vector<cv::Vec4i> hierarchy;
  findContours( GMMChannel.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE); 
  cv::Rect tempRect, floodRect;
  for( int i = 0; i< contours.size(); i++ )
  { 
    tempRect = boundingRect(contours[i]);
    float rectRatio;
    if(tempRect.width < tempRect.height)
    {
      rectRatio = (float)tempRect.width/(float)tempRect.height;
    }
    else
    {
      rectRatio = (float)tempRect.height/(float)tempRect.width;
    }
    if (tempRect.width > minBoundingRectDim && tempRect.width < maxBoundingRectDim && tempRect.height > minBoundingRectDim && tempRect.height < maxBoundingRectDim && rectRatio > minBoundingRectRatio)
    {
      float tempRectArea = (float)tempRect.width*(float)tempRect.height;
      cv::Mat tempMat1 = intensity(tempRect);
      cv::Scalar meanIntensity = mean(tempMat1);
      cv::Mat tempMat2 = GMMChannel(tempRect);
      cv::Scalar confidence = mean(tempMat2);
      
      // alternative using id of max intensity pixel as seed
      double max;
      //cv::Point maxPoint(tempRect.x+tempRect.width/2,tempRect.y+tempRect.height/2);
      //minMaxLoc(tempMat1, NULL, &max, NULL, &maxPoint);
      //maxPoint = ()
      floodFill(colorFrame.clone(), cv::Point(tempRect.x+tempRect.width/2,tempRect.y+tempRect.height/2), cvScalar(255,0,0), &floodRect, cvScalarAll(20), cvScalarAll(10), 8 | ( 255 << 8 ) | cv::FLOODFILL_FIXED_RANGE);
      rectangle(mything, floodRect, cv::Scalar( 0, 0, 255 ), 1, 4 );
/*
      // Using id of median intensity pixel as seed, center column
      cv::Mat sortedIndexes;
      cv::sortIdx(tempMat, sortedIndexes, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
      int medianRowIndex = sortedIndexes.at<uchar>(sortedIndexes.rows/2, sortedIndexes.cols/2);

      // debug
      //int medianIntensity = intensity.at<uchar>(tempRect.y+medianRowIndex, tempRect.x+tempRect.width/2);
      //cv::Point medianPoint = cv::Point(medianRowIndex,sortedIndexes.cols/2);
      //cout << "value: " << medianIntensity << " " << medianPoint << endl;
      floodFill(intensity.clone(), cv::Point(tempRect.x+tempRect.width/2, tempRect.y+medianRowIndex), cvScalar(255,0,0), &floodRect, cvScalarAll(30), cvScalarAll(40), 8 | ( 255 << 8 ) | cv::FLOODFILL_FIXED_RANGE);
*/
      float floodArea = (float)floodRect.width*(float)floodRect.height;
      float floodRatio;
      if(floodArea < tempRectArea)
      {
        floodRatio = floodArea/tempRectArea;
      }
      else
      {
        floodRatio = tempRectArea/floodArea;
      }
      //if(floodRatio > 0.1)
      //{
        rectangle(mything, tempRect, cv::Scalar( 0, 255, 255 ), 1, 4 );
        //cout << "Contour index: " << i << " confidence: " << confidence[0]/255 << " tempRectArea: " << tempRectArea << " floodArea: " << floodArea << " floodRatio: " << floodRatio << " rectRatio: " << rectRatio << endl;
        ColorLight tempDetectedColorLight;
        tempDetectedColorLight.color = color;
        tempDetectedColorLight.ROI = tempRect;
        tempDetectedColorLight.rectRatio = rectRatio;
        tempDetectedColorLight.floodRatio = floodRatio;
        tempDetectedColorLight.meanIntensity = meanIntensity[0]/255;
        tempDetectedColorLight.confidence = confidence[0]/255;
        detectedColorLights.push_back(tempDetectedColorLight); 
      //}
    }
  }
}

 // Function totalNumOfTrainingSamplesRows
 // Description:
 //     Function used to calculate the total amount of trainingpixels.
 //      Used for allocating the final trainingSample Mat for train em_model.

int GMM::findNumberOfTraningSamplePixels(){
    char file_name[150];
    int numberOfTraningSamplePixels=0;
    
    for (int i = 0; i < numberOfTrainingSamples; ++i)
    {
        sprintf(file_name,"data/redTraining1/%d.png",i);
        myTrainingImage = cv::imread(file_name);
        numberOfTraningSamplePixels += (myTrainingImage.rows*myTrainingImage.cols);
    }
    /*

    for (int i = 0; i < numberOfTrainingSamples; ++i)
    {
        sprintf(file_name,"data/greenTraining2/%d.png",i);
        myTrainingImage = cv::imread(file_name);
        numberOfTraningSamplePixels += (myTrainingImage.rows*myTrainingImage.cols);
    }
    
    for (int i = 0; i < numberOfTrainingSamples; ++i)
    {
        sprintf(file_name,"data/yellowTraining1/%d.png",i);
        myTrainingImage = cv::imread(file_name);
        numberOfTraningSamplePixels += (myTrainingImage.rows*myTrainingImage.cols);
    }
    
    for (int i = 0; i < 1; ++i)
    {
        sprintf(file_name,"data/other/%d.png",i);
        myTrainingImage = cv::imread(file_name);
        numberOfTraningSamplePixels += (myTrainingImage.rows*myTrainingImage.cols);
    }
    */
    return numberOfTraningSamplePixels;
}

// Function totalNumOfTrainingSamplesRows
// Description:
//      Function used to sort and save the training data. The YCrCb colorspace is splitted out,
//      in this code, whereof only the Cr and Cb Components are used for training.
//      Please note, that the same of number of trainingsamples is used here.
//
// Returns:
//      trainingSample - a cv::mat with all the training data with the Cr and Cb components
//                         from YCrCb color space. 
//
cv::Mat GMM::combineTrainingSamples(){

    cv::Mat trainingSample(findNumberOfTraningSamplePixels(),3,CV_32FC1); 
    int pixelIter=0;
    
    for (int k = 0; k < numberOfTrainingSamples; ++k)
    {   
        char file_name[150];
        sprintf(file_name,"data/redTraining1/%d.png",k);
        trainingImage = cv::imread(file_name);
        cv::Mat ycbcr;
        cvtColor(trainingImage,ycbcr,CV_BGR2YCrCb);
        for (int i = 0; i < trainingImage.rows; ++i)
        {
            for (int j = 0; j < trainingImage.cols; ++j)
            {
                int y = ycbcr.at<cv::Vec3b>(i,j)[0];
                int cb = ycbcr.at<cv::Vec3b>(i,j)[1];
                int cr = ycbcr.at<cv::Vec3b>(i,j)[2];

                trainingSample.at<float>(pixelIter,0) = (float)y;
                trainingSample.at<float>(pixelIter,1) = (float)cb;
                trainingSample.at<float>(pixelIter,2) = (float)cr;
                ++pixelIter;
            }
        }
    }
    
/*
    for (int k = 0; k < numberOfTrainingSamples; ++k)
    {   
        char file_name[150];
        sprintf(file_name,"data/greenTraining2/%d.png",k);
        trainingImage = cv::imread(file_name);
        cv::Mat ycbcr;
        cvtColor(trainingImage,ycbcr,CV_BGR2YCrCb);
        for (int i = 0; i < trainingImage.rows; ++i)
        {
            for (int j = 0; j < trainingImage.cols; ++j)
            {
                int y = ycbcr.at<cv::Vec3b>(i,j)[0];
                int cb = ycbcr.at<cv::Vec3b>(i,j)[1];
                int cr = ycbcr.at<cv::Vec3b>(i,j)[2];

                trainingSample.at<float>(pixelIter,0) = (float)y;
                trainingSample.at<float>(pixelIter,1) = (float)cb;
                trainingSample.at<float>(pixelIter,2) = (float)cr;
                ++pixelIter;
            }
        }
    }

	
    for (int k = 0; k < numberOfTrainingSamples; ++k)
    {   
        char file_name[150];
        sprintf(file_name,"data/yellowTraining1/%d.png",k);
        trainingImage = cv::imread(file_name);
        cv::Mat ycbcr;
        cvtColor(trainingImage,ycbcr,CV_BGR2YCrCb);
        for (int i = 0; i < trainingImage.rows; ++i)
        {
            for (int j = 0; j < trainingImage.cols; ++j)
            {
                int y = ycbcr.at<cv::Vec3b>(i,j)[0];
                int cb = ycbcr.at<cv::Vec3b>(i,j)[1];
                int cr = ycbcr.at<cv::Vec3b>(i,j)[2];

                trainingSample.at<float>(pixelIter,0) = (float)y;
                trainingSample.at<float>(pixelIter,1) = (float)cb;
                trainingSample.at<float>(pixelIter,2) = (float)cr;
                ++pixelIter;
            }
        }
    }

    for (int k = 0; k < 1; ++k)
    {   
        char file_name[150];
        sprintf(file_name,"data/other/%d.png",k);
        trainingImage = cv::imread(file_name);
        cv::Mat ycbcr;
        cvtColor(trainingImage,ycbcr,CV_BGR2YCrCb);
        for (int i = 0; i < trainingImage.rows; ++i)
        {
            for (int j = 0; j < trainingImage.cols; ++j)
            {
                int y = ycbcr.at<cv::Vec3b>(i,j)[0];
                int cb = ycbcr.at<cv::Vec3b>(i,j)[1];
                int cr = ycbcr.at<cv::Vec3b>(i,j)[2];

                trainingSample.at<float>(pixelIter,0) = (float)y;
                trainingSample.at<float>(pixelIter,1) = (float)cb;
                trainingSample.at<float>(pixelIter,2) = (float)cr;
                ++pixelIter;
            }
        }
    }
    */
    return trainingSample;
}

// Function totalNumOfTrainingSamplesRows
// Description:
//      Initialize the em_model with the trainingsamples.
//

void GMM::init(){
    colorModel->setCovarianceMatrixType(EM::COV_MAT_DIAGONAL); // COV_MAT_DIAGONAL || COV_MAT_SPHERICAL
    colorModel->setTermCriteria(cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 3000, 0.01));
    colorModel->setClustersNumber(2);
    cout << "Training..." << endl;
 	colorModel->trainEM(combineTrainingSamples());
    colorModel->save("colorModel");
}

// Function totalNumOfTrainingSamplesRows
// Description:
//      Predicts whether each pixels in the input frames belongs to the GMM of either
//      red or green.
//
// Arguments:
//      frame - input frame whereof all the pixels most be color segmented according
//              to the trained em_model.
 
