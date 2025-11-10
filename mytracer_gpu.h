// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#ifndef MYTRACER_GPU_H
#define MYTRACER_GPU_H

#include <vector>
#include "utils/Camera.h"
#include "utils/vec4.h"
#include "utils/Material.h"
#include "utils/Image.h"
#include "mydata.h"
#include "mybvh.h"
#include <cuda_runtime.h>

// C++ functions on host
void init_device();
Image launch_compute_image_device(vec4* d_pixels,
                                  int width,
                                  int height,
                                  const Camera& camera,
                                  const vec4* d_lightsPos,
                                  const vec4* d_lightsColor,
                                  int nLights,
                                  const vec4& background,
                                  const vec4& ambience,
                                  int max_depth,
                                  const Data* data,
                                  const BVH::BVHNodes_SoA* bvhNodes);

// Kernels and device functions
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

__global__ void adaptive_supersampling_device(vec4* pixels,
                                              vec4* tmpPixels,
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
                                              const BVH::BVHNodes_SoA *bvhNodes,
                                              const int subp,
                                              const int threshold);


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

__device__ bool intersect_triangle_device(const vec4& p0,
                                          const vec4& p1,
                                          const vec4& p2,
                                          const vec4& n,
                                          const vec4& vn0,
                                          const vec4& vn1,
                                          const vec4& vn2,
                                          const double& u0,
                                          const double& u1,
                                          const double& u2,
                                          const double& v0,
                                          const double& v1,
                                          const double& v2,
                                          const Ray &ray,
                                          vec4 &intersection_point,
                                          vec4 &intersection_normal,
                                          vec4 &intersection_diffuse,
                                          double &intersection_distance,
                                          const int & meshId,
                                          const Data *data);

__device__ bool intersectAABB(const Ray & ray, const vec4 & bb_min_, const vec4 & bb_max_, double & tmin_);

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
