
#include "vdispkernel.h"

using namespace std;

void gpuGenerateVdisp(unsigned char* const d_image,
                            unsigned int* d_vdispImage, 
                            size_t numRows, size_t numCols, size_t vdispNumRows, size_t vdispNumCols);
