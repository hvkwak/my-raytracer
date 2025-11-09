// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#ifndef MYTRACER_GPU_H
#define MYTRACER_GPU_H

#include "utils/Camera.h"
#include "utils/vec4.h"
#include "mydata.h"
#include <cuda_runtime.h>

__global__ void compute_image_device(vec4* pixels,
                                     const int width,
                                     const int height,
                                     const Camera camera,
                                     const vec4 *lightsPos,
                                     const vec4 *lightsColor,
                                     const int nLights,
                                     const vec4 background,
                                     const vec4 ambience,
                                     const int max_depth,
                                     const Data data);

__device__ vec4 trace_device(const Ray &ray,
                             const vec4 &background,
                             const vec4 &ambience,
                             const int &max_depth,
                             const int depth,
                             const Data & data);

__device__ bool intersect_scene_device();

#endif // MYTRACER_GPU_H
