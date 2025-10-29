#include "./common/common.h"
#include <cuda_runtime.h>
#include <stdio.h>
#include <ctime>


void cudaInit(void){
    int dev = 0;
    cudaDeviceProp deviceProp;
    CHECK(cudaGetDeviceProperties(&deviceProp, dev));
    printf("Using Device %d: %s\n", dev, deviceProp.name);
    CHECK(cudaSetDevice(dev));
}
