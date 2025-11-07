// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#ifndef MYTRACER_GPU_H
#define MYTRACER_GPU_H

#include "utils/Light.h"
#include "utils/Camera.h"
#include "utils/vec4.h"
#include "utils/Ray.h"
#include "mydata.h"
#include <cuda_runtime.h>

__global__ void traceOnGPU(vec4* pixels,
                           int width,
                           int height,
                           Camera camera,
                           Light* lights,
                           int numLights,
                           Data data,
                           vec4 background,
                           vec4 ambience,
                           int max_depth);

#endif // MYTRACER_GPU_H
