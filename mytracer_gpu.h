// ============================================================================
// Solutions/Implementations by Hyovin Kwak to the course
// Computer Graphics @ TU Dortmund (Instructor: Prof. Dr. Mario Botsch)
//
// Note: The original exercise codebase is not included in this repo.
// ============================================================================

#ifndef MYTRACER_GPU_H
#define MYTRACER_GPU_H

#ifdef CUDA_ENABLED
//#include <vector>
#include "utils/Camera.h"
#include "utils/vec4.h"
#include "utils/Material.h"
#include "utils/Image.h"
#include "mydata.h"
#include "mybvh.h"
#include <cuda_runtime.h>

// C++ functions on host
/**
 * @brief Initialize CUDA device
 */
void init_device();
/**
 * @brief Launch GPU kernels for raytracing computation
 * @param d_pixels Device pixel buffer
 * @param d_tmpPixels Temporary device pixel buffer for supersampling
 * @param d_image Device image output buffer
 * @param width Image width
 * @param height Image height
 * @param camera Camera configuration
 * @param d_lightsPos Device array of light positions
 * @param d_lightsColor Device array of light colors
 * @param nLights Number of lights
 * @param background Background color
 * @param ambience Ambient light color
 * @param max_depth Maximum ray recursion depth
 * @param data Scene data in SoA format
 * @param bvhNodes BVH acceleration structure
 */
void launch_compute_image_device(vec4* d_pixels,
                                 vec4* d_tmpPixels,
                                 vec4* d_image,
                                 const int& width,
                                 const int& height,
                                 const Camera& camera,
                                 const vec4* d_lightsPos,
                                 const vec4* d_lightsColor,
                                 const int& nLights,
                                 const vec4& background,
                                 const vec4& ambience,
                                 const int& max_depth,
                                 const Data* data,
                                 const BVH::BVHNodes_SoA* bvhNodes);

// Kernels and device functions
/**
 * @brief GPU kernel to compute raytraced image
 * @param pixels Output pixel buffer
 * @param width Image width
 * @param height Image height
 * @param camera Camera configuration
 * @param lightsPos Light positions
 * @param lightsColor Light colors
 * @param nLights Number of lights
 * @param background Background color
 * @param ambience Ambient light
 * @param max_depth Maximum recursion depth
 * @param data Scene data
 * @param bvhNodes BVH structure
 */
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

/**
 * @brief GPU kernel for adaptive supersampling
 * @param pixels Output pixel buffer
 * @param tmpPixels Input pixel buffer from first pass
 * @param width Image width
 * @param height Image height
 * @param camera Camera configuration
 * @param lightsPos Light positions
 * @param lightsColor Light colors
 * @param nLights Number of lights
 * @param background Background color
 * @param ambience Ambient light
 * @param max_depth Maximum recursion depth
 * @param data Scene data
 * @param bvhNodes BVH structure
 * @param subp Subpixel count
 * @param threshold Color difference threshold for supersampling
 */
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
                                              const double threshold);

/**
 * @brief Trace ray through scene (GPU version)
 * @param ray Ray to trace
 * @param background Background color
 * @param ambience Ambient light
 * @param max_depth Maximum recursion depth
 * @param lightsPos Light positions
 * @param lightsColor Light colors
 * @param nLights Number of lights
 * @param data Scene data
 * @param bvhNodes BVH structure
 * @return Computed color
 */
__device__ vec4 trace_device(const Ray &ray,
                             const vec4 &background,
                             const vec4 &ambience,
                             const int &max_depth,
                             const vec4* lightsPos,
                             const vec4* lightsColor,
                             const int nLights,
                             const Data *data,
                             const BVH::BVHNodes_SoA *bvhNodes);

/**
 * @brief Find closest scene intersection (GPU version)
 * @param ray Ray to test
 * @param data Scene data
 * @param bvhNodes BVH structure
 * @param material Material at intersection (output)
 * @param intersection_point Intersection point (output)
 * @param intersection_normal Intersection normal (output)
 * @param intersection_distance Intersection distance (output)
 * @return true if intersection found
 */
__device__ bool intersect_scene_device(const Ray & ray,
                                       const Data *data,
                                       const BVH::BVHNodes_SoA *bvhNodes,
                                       Material &material,
                                       vec4 &intersection_point,
                                       vec4 &intersection_normal,
                                       double &intersection_distance);

/**
 * @brief Traverse BVH for ray intersection (GPU version)
 * @param ray Ray to test
 * @param data Scene data
 * @param bvhNodes BVH structure
 * @param material Material at intersection (output)
 * @param intersection_point Intersection point (output)
 * @param intersection_normal Intersection normal (output)
 * @param intersection_distance Intersection distance (output)
 * @return true if intersection found
 */
__device__ bool intersectBVH_device(const Ray &ray,
                                    const Data *data,
                                    const BVH::BVHNodes_SoA *bvhNodes,
                                    Material &material,
                                    vec4 &intersection_point,
                                    vec4 &intersection_normal,
                                    double &intersection_distance);

/**
 * @brief Test ray-triangle intersection (GPU SoA version)
 * @param p0 First vertex position
 * @param p1 Second vertex position
 * @param p2 Third vertex position
 * @param n Face normal
 * @param vn0 First vertex normal
 * @param vn1 Second vertex normal
 * @param vn2 Third vertex normal
 * @param u0 First vertex U coordinate
 * @param u1 Second vertex U coordinate
 * @param u2 Third vertex U coordinate
 * @param v0 First vertex V coordinate
 * @param v1 Second vertex V coordinate
 * @param v2 Third vertex V coordinate
 * @param ray Ray to test
 * @param intersection_point Intersection point (output)
 * @param intersection_normal Intersection normal (output)
 * @param intersection_diffuse Intersection color (output)
 * @param intersection_distance Intersection distance (output)
 * @param meshId Mesh ID
 * @param data Scene data
 * @return true if intersection found
 */
__device__ bool intersect_triangle_device(const int& triIndex,
                                          const Ray &ray,
                                          vec4 &intersection_point,
                                          vec4 &intersection_normal,
                                          vec4 &intersection_diffuse,
                                          double &intersection_distance,
                                          const double &current_best_distance,
                                          int & meshId,
                                          const Data *data);

/**
 * @brief Test ray-AABB intersection (GPU version)
 * @param ray Ray to test
 * @param bb_min_ AABB minimum corner
 * @param bb_max_ AABB maximum corner
 * @param tmin_ Minimum intersection distance (output)
 * @return true if intersection found
 */
__device__ bool intersectAABB_device(const Ray & ray, const vec4 & bb_min_, const vec4 & bb_max_, double & tmin_);

/**
 * @brief Compute Phong lighting (GPU version)
 * @param point Intersection point
 * @param normal Surface normal
 * @param view View direction
 * @param material Material properties
 * @param data Scene data
 * @param bvhNodes BVH structure
 * @param background Background color
 * @param ambience Ambient light
 * @param lightsPos Light positions
 * @param lightsColor Light colors
 * @param nLights Number of lights
 * @return Computed color
 */
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

/**
 * @brief Compute diffuse term (GPU version)
 * @param point Surface point
 * @param normal Surface normal
 * @param lightPos Light position
 * @return Diffuse factor
 */
__device__ double diffuse_device(const vec4 &point, const vec4 &normal, const vec4 &lightPos);

/**
 * @brief Compute specular reflection term (GPU version)
 * @param point Surface point
 * @param normal Surface normal
 * @param view View direction
 * @param lightPos Light position
 * @return Specular factor
 */
__device__ double reflection_device(const vec4 &point, const vec4 &normal, const vec4 &view, const vec4 &lightPos);
#endif // CUDA_ENABLED
#endif // MYTRACER_GPU_H
