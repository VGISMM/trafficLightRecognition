#include "Kalman.h"

Kalman::Kalman(){

}

void Kalman::initKalman(float x, float y)
{
    // Instantate Kalman Filter with
    // 4 dynamic parameters and 2 measurement parameters,
    // where my measurement is: 2D location of object,
    // and dynamic is: 2D location and 2D velocity.
    KF.init(4, 2, 0);

    measurement.at<float>(0, 0) = x;
    measurement.at<float>(1, 0) = y;

    KF.statePre.setTo(0);
    KF.statePre.at<float>(0, 0) = x;
    KF.statePre.at<float>(1, 0) = y;

    KF.statePost.setTo(0);
    KF.statePost.at<float>(0, 0) = x;
    KF.statePost.at<float>(1, 0) = y; 
    KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1);

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, cv::Scalar::all(0.004)); //adjust this for faster convergence - but higher noise
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, cv::Scalar::all(.1));
}

void Kalman::kalmanPredict() 
{
    prediction = KF.predict();
   // Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

}

void Kalman::kalmanCorrect(float x, float y)
{
    measurement(0) = x;
    measurement(1) = y;
    estimated = KF.correct(measurement);
    //Point statePt(estimated.at<float>(0),estimated.at<float>(1));

}