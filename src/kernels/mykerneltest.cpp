#include "mykernel.h"
#include "mykernel.cu"

using namespace std;

void your_rgba_to_greyscale(unsigned char* const d_image,
                            unsigned int* d_vdispImage, 
                            size_t numRows, size_t numCols, size_t vdispNumRows, size_t vdispNumCols)
{
  const int thread = 32;
  const dim3 blockSize( thread, thread, 1);
  const dim3 gridSize( ceil(numRows/(float)thread), ceil(numCols/(float)thread), 1);
  
  //rgba_to_greyscale<<<((numRows*numCols)/32), 32>>>(d_image, d_vdispImage, numRows, numCols, vdispNumRows, vdispNumCols);
  rgba_to_greyscale<<<gridSize, blockSize>>>(d_image, d_vdispImage, numRows, numCols, vdispNumRows, vdispNumCols);
  cudaDeviceSynchronize();
}
