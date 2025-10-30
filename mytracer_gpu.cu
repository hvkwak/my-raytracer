#include "./common/common.h"
#include <cuda_runtime.h>
#include <stdio.h>
#include <ctime>

#include "Raytracer.h"
#include "utils/vec3.h"
#include "utils/Material.h"
#include "utils/Object.h"

void Raytracer::cudaInit(void){
    // Init Device
    int dev = 0;
    cudaDeviceProp deviceProp;
    CHECK(cudaGetDeviceProperties(&deviceProp, dev));
    printf("Using Device %d: %s\n", dev, deviceProp.name);
    CHECK(cudaSetDevice(dev));
}
