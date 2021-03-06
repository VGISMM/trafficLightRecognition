#include "Disparity.h"
Disparity::Disparity() 
{
	dbf = cv::cuda::createDisparityBilateralFilter(96,3,1);   
	//gpubm = new cv::cuda::StereoBM();
	//gpubm->preset = cv::cuda::StereoBM_GPU::BASIC_PRESET;
  gpubm = cv::cuda::createStereoBM(96,9);
  //bm->setROI1(roi1);
  //bm->setROI2(roi2);
  gpubm->setPreFilterCap(31);
  gpubm->setBlockSize(9);
  gpubm->setMinDisparity(0);
  //bm->setNumDisparities(numberOfDisparities);
  gpubm->setTextureThreshold(3);
  gpubm->setUniquenessRatio(15);
  gpubm->setSpeckleWindowSize(100);
  gpubm->setSpeckleRange(32);
  gpubm->setDisp12MaxDiff(1);

  sgbm = cv::StereoSGBM::create(0,96,3);
  sgbm->setPreFilterCap(63);
  int sgbmWinSize = 9;
  sgbm->setBlockSize(sgbmWinSize);
  sgbm->setP1(8*3*sgbmWinSize*sgbmWinSize);
  sgbm->setP2(32*3*sgbmWinSize*sgbmWinSize);
  sgbm->setMinDisparity(0);
  //sgbm->setNumDisparities(numberOfDisparities);
  sgbm->setUniquenessRatio(10);
  sgbm->setSpeckleWindowSize(100);
  sgbm->setSpeckleRange(32);
  sgbm->setDisp12MaxDiff(1);
  sgbm->setMode(cv::StereoSGBM::MODE_HH); //StereoSGBM::MODE_SGBM
/*
  sgbm.preFilterCap = 1;
  sgbm.SADWindowSize = 9;
  sgbm.P1 = 8*3*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.P2 = 32*3*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.minDisparity = 0;
  sgbm.numberOfDisparities = 96;
  //sgbm.uniquenessRatio = 10;
  sgbm.speckleWindowSize = 2;
  sgbm.speckleRange = 3;
  //sgbm.disp12MaxDiff = 1;
  //sgbm.fullDP = 1;
  */
}

void Disparity::GPUBMdisparity() 
{ 
  cv::cuda::GpuMat grayLOIGPU, grayROIGPU;
  cv::cuda::GpuMat dispLRGPU, dispRLGPU;
  cv::cuda::GpuMat imgLOIGPUflip, imgROIGPUflip, dispRLGPUflip;
  cv::cuda::GpuMat dispRLLRGPU;

  //gpubm->ndisp = 80;
  //gpubm->winSize = 9;
  //gpubm->avergeTexThreshold = 3;
  //-----------------------------------Upper disp-------------------------------------------------
  cv::cuda::cvtColor(imgLOIGPU(cv::Rect(cv::Point(0,0), cv::Size(IMAGEWIDTH, IMAGEHEIGHT/2))), grayLOIGPU, cv::COLOR_BGR2GRAY);
  cv::cuda::cvtColor(imgROIGPU(cv::Rect(cv::Point(0,0), cv::Size(IMAGEWIDTH, IMAGEHEIGHT/2))), grayROIGPU, cv::COLOR_BGR2GRAY);
	
  gpubm->setPreFilterCap(31);
  gpubm->setBlockSize(9);
  gpubm->setTextureThreshold(3);
  gpubm->setUniquenessRatio(15);
  gpubm->setSpeckleWindowSize(100);
  gpubm->setSpeckleRange(32);

  // LR BM
  auto bmstart = chrono::steady_clock::now();
	gpubm->compute(grayLOIGPU, grayROIGPU, dispLRGPU);
  auto bmend = chrono::steady_clock::now();
  // Store the time difference between start and end
  auto bmdiff = bmend - bmstart;
  std::cout << "Stereo coorespondence: " << chrono::duration <double, milli> (bmdiff).count() << " ms" << endl;
	//cv::gpu::normalize(dispLRGPU, dispLRGPU, 0, 255, CV_MINMAX, CV_8U);

	// RL BM
	cv::cuda::flip(grayLOIGPU,imgLOIGPUflip,1);
	cv::cuda::flip(grayROIGPU,imgROIGPUflip,1);
	gpubm->compute(imgROIGPUflip, imgLOIGPUflip, dispRLGPUflip);
	//cv::gpu::normalize(dispRLGPUflip, dispRLGPUflip, 0, 255, CV_MINMAX, CV_8U);
	cv::cuda::flip(dispRLGPUflip,dispRLGPU,1);

  // RLLR consistency check
  cv::cuda::min(dispLRGPU,dispRLGPU,upperDispFinished); //dispRLLRGPU);
  
  // bilateral filtering
  //dbf->operator()(dispRLLRGPU, imgLOIGPU, upperDispFinished); 
//-----------------------------------Lower disp-------------------------------------------------
  //gpubm->ndisp = 96;
  //gpubm->winSize = 7;
  //gpubm->avergeTexThreshold = 1;

  cv::cuda::cvtColor(imgLOIGPU(cv::Rect(cv::Point(0,IMAGEHEIGHT/2), cv::Size(IMAGEWIDTH, IMAGEHEIGHT/2))), grayLOIGPU, cv::COLOR_BGR2GRAY);
  cv::cuda::cvtColor(imgROIGPU(cv::Rect(cv::Point(0,IMAGEHEIGHT/2), cv::Size(IMAGEWIDTH, IMAGEHEIGHT/2))), grayROIGPU, cv::COLOR_BGR2GRAY);

  gpubm->setPreFilterCap(5);
  gpubm->setBlockSize(5);
  gpubm->setTextureThreshold(0);
  gpubm->setUniquenessRatio(1);
  gpubm->setSpeckleWindowSize(9);
  gpubm->setSpeckleRange(32);

  // LR BM
  gpubm->compute(grayLOIGPU, grayROIGPU, dispLRGPU);
  
  // RL BM
  cv::cuda::flip(grayLOIGPU,imgLOIGPUflip,1);
  cv::cuda::flip(grayROIGPU,imgROIGPUflip,1);
  gpubm->compute(imgROIGPUflip, imgLOIGPUflip, dispRLGPUflip);
  //cv::gpu::normalize(dispRLGPUflip, dispRLGPUflip, 0, 255, CV_MINMAX, CV_8U);
  cv::cuda::flip(dispRLGPUflip,dispRLGPU,1);

  // RLLR consistency check
  cv::cuda::min(dispLRGPU,dispRLGPU,lowerDispFinished); //dispRLLRGPU);
  
  // bilateral filtering
  //dbf->operator()(dispRLLRGPU, imgLOIGPU, lowerDispFinished); 
}

void Disparity::SGBMdisparity() 
{ 
  sgbm->compute(imgL, imgR, dispLR);
  dispLR.convertTo(dispLR, CV_8U, 255/(96*16.));
  flip(imgL, imgL, 1);
  flip(imgR, imgR, 1);
  sgbm->compute(imgR, imgL, dispRL);
  dispRL.convertTo(dispRL, CV_8U, 255/(96*16.));
  flip(dispRL, dispRL, 1);
  cv::min(dispLR,dispRL,dispRLLR);
}

void Disparity::temporalDisparity() 
{
    temporalCombinedDisps.setTo(cv::Scalar(0));
    //cv::Mat temporalCombinedDisps=cv::Mat::zeros(combinedDisps.rows,combinedDisps.cols,combinedDisps.type());
    for (int j = 0; j < combinedDisps.cols; j++ ) {
        for (int i = 0; i < combinedDisps.rows; i++) {
            if(abs(combinedDisps.at<uchar>(i, j)-temporalDisp.at<uchar>(i, j)) <= 20) 
            {
                temporalCombinedDisps.at<uchar>(i, j) = combinedDisps.at<uchar>(i, j);
            }
        }
    }
}

void Disparity::vDispThresholdedImage(cv::Mat dispImg, float slope, float intersection, float thresholdOffset) 
{
// Create V disparity thresholded obstacleImage 
    obstacleImage.setTo(cv::Scalar(0));
    for (int j = 0; j < dispImg.cols; j++ ) {
        for (int i = 0; i < dispImg.rows; i++) {
            if (dispImg.at<uchar>(i, j) > (((i-(intersection))/(slope))+thresholdOffset))
            {  
                obstacleImage.at<uchar>(i, j) = dispImg.at<uchar>(i, j);
            } 
            if (dispImg.at<uchar>(i, j) > 220)
            {
                obstacleImage.at<uchar>(i, j) = 0;
            }
        }
    }
    //cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
    //morphologyEx( obstacleImage, obstacleImage, cv::MORPH_CLOSE, sel, cv::Point(-1,-1), 1 );
}

void Disparity::removeDarkRegions(cv::Mat orgImgLoi) 
{   
    postDarkImage.setTo(cv::Scalar(0));
    for (int j = 0; j < orgImgLoi.cols; j++ ) {
        for (int i = 0; i < orgImgLoi.rows; i++) {
            pixelValueBGR = orgImgLoi.at<cv::Vec3b>(i,j);    
            pixelValueBlue = pixelValueBGR.val[0];
            pixelValueGreen = pixelValueBGR.val[1];
            pixelValueRed = pixelValueBGR.val[2];

            if ( ((pixelValueBlue < 60) && (pixelValueGreen < 60) && (pixelValueRed > 160)) )//|| ((pixelValueBlue > 180) && (pixelValueGreen > 180) && (pixelValueRed > 180)) )
            {   
                postDarkImage.at<uchar>(i, j) = 255;
            }
        }
    }
}

void Disparity::generateVdisp(cv::Mat h_disp) 
{
  unsigned char *inputImage; //host
  unsigned char *d_image;
  unsigned int *d_vdispImage; // device

  cudaFree(0);

   //This shouldn't ever happen given the way the images are created
  //at least based upon my limited understanding of OpenCV, but better to check
  if (!h_disp.isContinuous()) {
    std::cerr << "Images aren't continuous!! Exiting." << std::endl;
    exit(1);
  }
  //unsigned char* data = dispRLGPU.ptr<unsigned char>();

  const size_t imgNumPixels = IMAGEHEIGHT * IMAGEWIDTH;
  const size_t vdispNumPixels = IMAGEHEIGHT * 255;
  //allocate memory on the device for both input and output cudaMalloc((void **)&d_a, size);
  cudaMalloc((void **) &d_image, sizeof(unsigned char) * imgNumPixels);
  cudaMalloc((void **) &d_vdispImage, sizeof(unsigned int) * vdispNumPixels);
  
  cudaMemset(d_vdispImage, 0, sizeof(unsigned int) * vdispNumPixels); //make sure no memory is left laying around
  //copy input array to the GPU
  cudaMemcpy(d_image, h_disp.ptr<unsigned char>(0), sizeof(unsigned char) * imgNumPixels, cudaMemcpyHostToDevice);
  //d_rgbaImage__ = *d_image;
  //d_greyImage__ = *d_greyImage;
  auto start = chrono::steady_clock::now();
  //call the grayscale code
  gpuGenerateVdisp(d_image, d_vdispImage, IMAGEHEIGHT, IMAGEWIDTH, IMAGEHEIGHT, 255 );

 // cudaDeviceSynchronize(); 
  auto end = chrono::steady_clock::now();
  // Store the time difference between start and end
  auto diff = end - start;

  //copy the output back to the host
  cudaMemcpy(vdispU16.ptr<unsigned int>(0), d_vdispImage, sizeof(unsigned int) * vdispNumPixels, cudaMemcpyDeviceToHost);

  vdispU16.convertTo(vdisp, CV_8UC1);
  
  //output the image
  cout << "GPU time: " << chrono::duration <double, milli> (diff).count() << " ms" << endl;
  //cleanup
  cudaFree(d_image);
  cudaFree(d_vdispImage);
}

void Disparity::generateVdispCPU(cv::Mat dispImg) 
{
    //cv::Mat Vdisp = cv::Mat::zeros(dispImg.rows, 255, CV_8UC1);
    vdispCPU.setTo(cv::Scalar(0));
    for (int j = 0; j < dispImg.cols; j++ ) 
    {
        for (int i = 0; i < dispImg.rows; i++) 
        {
            if(dispImg.at<uchar>(i, j) > 0 && dispImg.at<uchar>(i, j) < 225)
            {
                if(vdispCPU.at<uchar>(i, dispImg.at<uchar>(i, j)) < 255 ){
                    vdispCPU.at<uchar>(i, dispImg.at<uchar>(i, j))++;
                }
            } 
        }
    }
    //return Vdisp;
}

