#include "emGMM.h"
emGMM::emGMM(){

}

/* Function totalNumOfTrainingSamplesRows
 * Description:
 *      Function used to calculate the total amount of trainingpixels.
 *      Used for allocating the final trainingSample Mat for train em_model.
 */
void emGMM::totalNumOfTrainingSamplesRows(){
    char file_name[150];
    totalTraningSamplesRows=0;
    for (int i = 0; i < numberOfTrainingSamples; ++i)
    {
        sprintf(file_name,"data/redTraining/%d.png",i);
        myTrainingImage = cv::imread(file_name);
        totalTraningSamplesRows = totalTraningSamplesRows + (myTrainingImage.rows*myTrainingImage.cols);
    }
    for (int i = 0; i < numberOfTrainingSamples; ++i)
    {
        sprintf(file_name,"data/greenTraining/%d.png",i);
        myTrainingImage = cv::imread(file_name);
        totalTraningSamplesRows = totalTraningSamplesRows + (myTrainingImage.rows*myTrainingImage.cols);
    }
}

/* Function totalNumOfTrainingSamplesRows
 * Description:
 *      Function used to sort and save the training data. The YCrCb colorspace is splitted out,
 *      in this code, whereof only the Cr and Cb Components are used for training.
 *      Please note, that the same of number of trainingsamples is used here.
 *
 * Returns:
 *      trainingSample - a cv::mat with all the training data with the Cr and Cb components
 *                         from YCrCb color space. 
 */
cv::Mat emGMM::createTrainingSamplesYCbCr(){
	totalNumOfTrainingSamplesRows();
    cv::Mat trainingSample(totalTraningSamplesRows,2,CV_32FC1); 
    int currentIter=0;
    for (int k = 0; k < numberOfTrainingSamples; ++k)
    {   
        char file_name[150];
        sprintf(file_name,"data/redTraining/%d.png",k);

        trainingImage = cv::imread(file_name);
        cv::Mat ycbcr;
        cvtColor(trainingImage,ycbcr,CV_BGR2YCrCb);

        for (int i = 0; i < trainingImage.rows; ++i)
        {
            for (int j = 0; j < trainingImage.cols; ++j)
            {
                //int y = ycbcr.at<cv::Vec3b>(i,j)[0];
                int cb = ycbcr.at<cv::Vec3b>(i,j)[1];
                int cr = ycbcr.at<cv::Vec3b>(i,j)[2];

                //trainingSample.at<float>(currentIter,0) = (float)y;
                trainingSample.at<float>(currentIter,0) = (float)cb;
                trainingSample.at<float>(currentIter,1) = (float)cr;
                ++currentIter;
            }
        }
    }
    for (int k = 0; k < numberOfTrainingSamples; ++k)
    {   
        char file_name[150];
        sprintf(file_name,"data/greenTraining/%d.png",k);
        trainingImage = cv::imread(file_name);
        cv::Mat ycbcr;
        cvtColor(trainingImage,ycbcr,CV_BGR2YCrCb);
        for (int i = 0; i < trainingImage.rows; ++i)
        {
            for (int j = 0; j < trainingImage.cols; ++j)
            {
                //int y = ycbcr.at<cv::Vec3b>(i,j)[0];
                int cb = ycbcr.at<cv::Vec3b>(i,j)[1];
                int cr = ycbcr.at<cv::Vec3b>(i,j)[2];

                //trainingSample.at<float>(currentIter,0) = (float)y;
                trainingSample.at<float>(currentIter,0) = (float)cb;
                trainingSample.at<float>(currentIter,1) = (float)cr;
                ++currentIter;
            }
        }
    }
    return trainingSample;
}

/* Function totalNumOfTrainingSamplesRows
 * Description:
 *      Initialize the em_model with the trainingsamples.
 */
void emGMM::init(){
    params.nclusters = 4;
    params.cov_mat_type       = CvEM::COV_MAT_SPHERICAL;
    params.start_step         = CvEM::START_AUTO_STEP;
    params.term_crit.max_iter = 30000;
    params.term_crit.epsilon  = 0.001;
    params.term_crit.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;

    cv::Mat samples = createTrainingSamplesYCbCr();

    em_model.train( samples, cv::Mat(), params);
}

/* Function totalNumOfTrainingSamplesRows
 * Description:
 *      Predicts whether each pixels in the input frames belongs to the GMM of either
 *      red or green.
 *
 * Arguments:
 *      frame - input frame whereof all the pixels most be color segmented according
                to the trained em_model.
 */
void emGMM::predictFrame(cv::Mat frame){
	cvtColor(frame,convertedYCrCb,CV_BGR2YCrCb);

    imgOut = cv::Mat::zeros( cv::Size( frame.cols, frame.rows ), CV_8UC3 );

    cv::Mat sample( 1, 2, CV_32FC1 );
    
    for( int i = 0; i < frame.rows; i++ )
    {
        for(int j = 0; j < frame.cols; j++ )
        {
            //int Y = convertedYCrCb.at<cv::Vec3b>(i,j)[0];
            int Cr = convertedYCrCb.at<cv::Vec3b>(i,j)[1];
            int Cb = convertedYCrCb.at<cv::Vec3b>(i,j)[2];

            //sample.at<float>(0) = (float)Y;
            sample.at<float>(0) = (float)Cr;
            sample.at<float>(1) = (float)Cb;

            int response = cvRound(em_model.predict( sample ));

            if (response==0){

            } //Trash cluster

            if (response==1)
            {
                imgOut.at<cv::Vec3b>(i,j) = frame.at<cv::Vec3b>(i,j);
            }

            if (response==2)
            {
               imgOut.at<cv::Vec3b>(i,j) = frame.at<cv::Vec3b>(i,j);
            }
            if (response==3)
            {
                 
            }
        
        }
    }
    cvtColor(imgOut,imgOut,CV_RGB2GRAY);
    threshold( imgOut, EmGmmAll, 1, 255,0 );

}
