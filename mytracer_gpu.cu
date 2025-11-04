// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#include "./common/common.h"
#include <cuda_runtime.h>
#include <stdio.h>
#include <ctime>

#include "Raytracer.h"
#include "utils/vec3.h"
#include "utils/Material.h"
#include "utils/Object.h"

void Raytracer::cudaInit(void){
    // Initialize CUDA device
    int dev = 0;
    cudaDeviceProp deviceProp;
    CHECK(cudaGetDeviceProperties(&deviceProp, dev));
    printf("Using Device %d: %s\n", dev, deviceProp.name);
    CHECK(cudaSetDevice(dev));
}
