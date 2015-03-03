
#include <cuda.h>
#include <cuda_runtime.h>
#include "vdispkernel.h"

__global__
void vdispKernel(unsigned char* const image,
                       unsigned int* vdispImage,
                       int numRows, int numCols, int vdispNumRows, int vdispNumCols)
{
  int index_x = blockIdx.x * blockDim.x + threadIdx.x;
  int index_y = blockIdx.y * blockDim.y + threadIdx.y;

  // map the two 2D indices to a single linear, 1D index
  int grid_width = gridDim.x * blockDim.x;
  int index = index_y * grid_width + index_x;
  
  //vdispImage[index] = image[index];
  
  int vdispIdx = (index/(numCols-1)*255) + image[index];
  atomicAdd(&vdispImage[vdispIdx],1);

  /*
  if(image[index]<240)
  {
    int vdispIdx = (index/(numCols-1)*255) + image[index];
    // write out the final result
    if (vdispImage[vdispIdx] < 255)
    {
      atomicAdd(&vdispImage[vdispIdx],1);
    }
  }
  */

  
}

void launchVdispKernel(unsigned char* const d_image,
                            unsigned int* d_vdispImage, 
                            size_t numRows, size_t numCols, size_t vdispNumRows, size_t vdispNumCols)
{
  const int thread = 32;
  const dim3 blockSize( thread, thread, 1);
  const dim3 gridSize( ceil(numRows/(float)thread), ceil(numCols/(float)thread), 1);
  
  //rgba_to_greyscale<<<((numRows*numCols)/32), 32>>>(d_image, d_vdispImage, numRows, numCols, vdispNumRows, vdispNumCols);
  vdispKernel<<<gridSize, blockSize>>>(d_image, d_vdispImage, numRows, numCols, vdispNumRows, vdispNumCols);
  cudaDeviceSynchronize();
}