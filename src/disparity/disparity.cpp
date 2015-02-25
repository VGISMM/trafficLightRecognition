#include "Disparity.h"
Disparity::Disparity() 
{
	dbf = new cv::gpu::DisparityBilateralFilter();   
	gpubm = new cv::gpu::StereoBM_GPU();
	gpubm->preset = cv::gpu::StereoBM_GPU::BASIC_PRESET;
	gpubm->ndisp = 64;
	gpubm->winSize = 11;
}

void Disparity::disparityImages() 
{ 
	// LR BM
    auto bmstart = chrono::steady_clock::now();
	gpubm->operator()(imgLOIGPU, imgROIGPU, dispLRGPU);
    auto bmend = chrono::steady_clock::now();
    // Store the time difference between start and end
    auto bmdiff = bmend - bmstart;
    std::cout << "Stereo coorespondence: " << chrono::duration <double, milli> (bmdiff).count() << " ms" << endl;
	cv::gpu::normalize(dispLRGPU, dispLRGPU, 0, 255, CV_MINMAX, CV_8U);

	// RL BM
	cv::gpu::flip(imgLOIGPU,imgLOIGPUflip,1);
	cv::gpu::flip(imgROIGPU,imgROIGPUflip,1);
	gpubm->operator()(imgROIGPUflip, imgLOIGPUflip, dispRLGPUflip);
	cv::gpu::normalize(dispRLGPUflip, dispRLGPUflip, 0, 255, CV_MINMAX, CV_8U);
	cv::gpu::flip(dispRLGPUflip,dispRLGPU,1);

    // RLLR consistency check
    cv::gpu::min(dispLRGPU,dispRLGPU,dispRLLRGPU);
    dbf->operator()(dispRLLRGPU, imgLOIGPU, dispFinished);

    
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

void Disparity::vDispThresholdedImage(float slope, float intersection, float thresholdOffset) 
{
// Create V disparity thresholded obstacleImage 
    obstacleImage.setTo(cv::Scalar(0));
    for (int j = 0; j < postDarkImage.cols; j++ ) {
        for (int i = 0; i < postDarkImage.rows; i++) {
            if (postDarkImage.at<uchar>(i, j) > (((i-(intersection))/(slope))+thresholdOffset))
            {  
                obstacleImage.at<uchar>(i, j) = postDarkImage.at<uchar>(i, j);
            } 
            if (postDarkImage.at<uchar>(i, j) > 220)
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
/*
void Disparity::reduceNumberOfBins(cv::Mat dispImg) 
{   
    reducedImg.setTo(cv::Scalar(0));
    //cv::Mat reducedImg = cv::Mat::zeros(dispImg.rows, dispImg.cols, CV_8UC1);
    for (int j = 0; j < dispImg.cols; j++ ) 
    {
        for (int i = 0; i < dispImg.rows; i++) 
        {
            reducedImg.at<uchar>(i, j) = dispImg.at<uchar>(i, j)/8;
        }
    }
    //return reducedImg;
}
*/
void Disparity::generateVdispAlt() 
{
  unsigned char *inputImage; //host
  unsigned char *d_image;
  unsigned int *d_vdispImage; // device

  //cudaFree(0);

   //This shouldn't ever happen given the way the images are created
  //at least based upon my limited understanding of OpenCV, but better to check
  if (!dispFinished.isContinuous()) {
    std::cerr << "Images aren't continuous!! Exiting." << std::endl;
    exit(1);
  }
  //unsigned char* data = dispRLGPU.ptr<unsigned char>();

  const size_t imgNumPixels = imageHeight * imageWidth;
  const size_t vdispNumPixels = imageHeight * 255;
  //allocate memory on the device for both input and output cudaMalloc((void **)&d_a, size);
  cudaMalloc((void **) &d_image, sizeof(unsigned char) * imgNumPixels);
  cudaMalloc((void **) &d_vdispImage, sizeof(unsigned int) * vdispNumPixels);
  
  cudaMemset(d_vdispImage, 0, sizeof(unsigned int) * vdispNumPixels); //make sure no memory is left laying around
  //copy input array to the GPU
  cudaMemcpy(d_image, dispFinished.ptr<unsigned char>(0), sizeof(unsigned char) * imgNumPixels, cudaMemcpyHostToDevice);
  //d_rgbaImage__ = *d_image;
  //d_greyImage__ = *d_greyImage;
  auto start = chrono::steady_clock::now();
  //call the grayscale code
  your_rgba_to_greyscale(d_image, d_vdispImage, imageHeight, imageWidth, imageHeight, 255 );

 // cudaDeviceSynchronize(); 
  auto end = chrono::steady_clock::now();
  // Store the time difference between start and end
  auto diff = end - start;

  //copy the output back to the host
  cudaMemcpy(vdisp.ptr<unsigned int>(0), d_vdispImage, sizeof(unsigned int) * vdispNumPixels, cudaMemcpyDeviceToHost);

  //output the image
  cv::imwrite("testGPU.png", vdisp);
  cout << "GPU time: " << chrono::duration <double, milli> (diff).count() << " ms" << endl;
  //cleanup
  cudaFree(d_image);
  cudaFree(d_vdispImage);
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

  const size_t imgNumPixels = imageHeight * imageWidth;
  const size_t vdispNumPixels = imageHeight * 255;
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
  your_rgba_to_greyscale(d_image, d_vdispImage, imageHeight, imageWidth, imageHeight, 255 );

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

void Disparity::generateUdisp(cv::Mat dispImg) 
{
    Udisp.setTo(cv::Scalar(0));
    //cv::Mat Udisp = cv::Mat::zeros(255, dispImg.cols, CV_8UC1);
    for (int j = 0; j < dispImg.cols; j++ ) 
    {
        for (int i = 0; i < dispImg.rows; i++) 
        {
            if(dispImg.at<uchar>(i, j) > 10 && dispImg.at<uchar>(i, j) < 235)
            {
                if(Udisp.at<uchar>(dispImg.at<uchar>(i, j), j) < 255 ){
                    Udisp.at<uchar>(dispImg.at<uchar>(i, j), j)++;
                }
            } 
        }
    }
}

/*
void Disparity::generateVdispPlanes(cv::Mat dispImg) 
{   
    VdispPlanes.setTo(cv::Scalar(0));
    //cv::Mat VdispPlanes = cv::Mat::zeros(dispImg.rows, dispImg.cols, CV_8UC1);
    for (int VdispRow = 0; VdispRow < Vdisp.rows; VdispRow++) 
    {
        for (int VdispCol = 0; VdispCol < Vdisp.cols; VdispCol++ ) 
        {
            if(Vdisp.at<uchar>(VdispRow, VdispCol) > (4*((VdispCol+1)/2)))
            //if(Vdisp.at<uchar>(VdispRow-1, VdispCol) + Vdisp.at<uchar>(VdispRow, VdispCol) + Vdisp.at<uchar>(VdispRow+1, VdispCol) > 60)
            {
                for (int dispImgCol = 0; dispImgCol < dispImg.cols; dispImgCol++ ) 
                {
                    if(dispImg.at<uchar>(VdispRow, dispImgCol) == VdispCol)
                    {
                        VdispPlanes.at<uchar>(VdispRow, dispImgCol) = VdispCol;
                    }
                }
            } 
        }
    }
    //return VdispPlanes;
}

void Disparity::generateUdispPlanes(cv::Mat dispImg) 
{
    UdispPlanes.setTo(cv::Scalar(0));
    //cv::Mat UdispPlanes = cv::Mat::zeros(dispImg.rows, dispImg.cols, CV_8UC1);
    for (int UdispRow = 0; UdispRow < Udisp.rows; UdispRow++) 
    {
        for (int UdispCol = 0; UdispCol < Udisp.cols; UdispCol++ ) 
        {
            if(Udisp.at<uchar>(UdispRow, UdispCol) > (1*((UdispRow+1)/2)))
            {
                for (int dispImgRow = 0; dispImgRow < dispImg.rows; dispImgRow++ ) 
                {
                    if(dispImg.at<uchar>(dispImgRow, UdispCol) == UdispRow)
                    {
                        UdispPlanes.at<uchar>(dispImgRow, UdispCol) = UdispRow;
                    }
                }
            } 
        }
    }
    //return UdispPlanes;
}
*/

