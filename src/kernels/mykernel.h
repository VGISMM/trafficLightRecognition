#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace std;

extern void your_rgba_to_greyscale(unsigned char* const d_image,
                            unsigned int* const d_vdispImage, 
                            size_t numRows, size_t numCols, size_t vdispNumRows, size_t vdispNumCols);
