#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <opencv2/opencv.hpp>


using namespace std;

void launchVdispKernel(unsigned char* const d_image,
                            unsigned int* d_vdispImage, 
                            size_t numRows, size_t numCols, size_t vdispNumRows, size_t vdispNumCols);
