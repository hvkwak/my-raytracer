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
#include "utils/Material.h"
#include "mydata.h"
#include "mybvh.h"
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
                                     const Data *data,
                                     const BVH::BVHNodes_SoA *bvhNodes);

__device__ vec4 trace_device(const Ray &ray,
                             const vec4 &background,
                             const vec4 &ambience,
                             const int &max_depth,
                             const vec4* lightsPos,
                             const vec4* lightsColor,
                             const int nLights,
                             const Data *data,
                             const BVH::BVHNodes_SoA *bvhNodes);

__device__ bool intersect_scene_device(const Ray & ray,
                                       const Data *data,
                                       const BVH::BVHNodes_SoA *bvhNodes,
                                       Material &material,
                                       vec4 &intersection_point,
                                       vec4 &intersection_normal,
                                       double &intersection_distance);

__device__ bool intersectBVH_device(const Ray &ray,
                                    const Data *data,
                                    const BVH::BVHNodes_SoA *bvhNodes,
                                    Material &material,
                                    vec4 &intersection_point,
                                    vec4 &intersection_normal,
                                    double &intersection_distance);

__device__ vec4 lighting_device(const vec4 &point,
                                const vec4 &normal,
                                const vec4 &view,
                                const Material &material,
                                const Data *data,
                                const BVH::BVHNodes_SoA *bvhNodes,
                                const vec4 &background,
                                const vec4 &ambience,
                                const vec4* lightsPos,
                                const vec4* lightsColor,
                                const int nLights);

__device__ double diffuse_device(const vec4 &point, const vec4 &normal, const vec4 &lightPos);
__device__ double reflection_device(const vec4 &point, const vec4 &normal, const vec4 &view, const vec4 &lightPos);
#endif // MYTRACER_GPU_H
