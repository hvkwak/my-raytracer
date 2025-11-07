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
#include <cfloat>

#include "Raytracer.h"
#include "utils/vec4.h"
#include "utils/Material.h"
#include "utils/Object.h"
#include "utils/Light.h"
#include "utils/Ray.h"

void Raytracer::cudaInit(void){
    // Initialize CUDA device
    int dev = 0;
    cudaDeviceProp deviceProp;
    CHECK(cudaGetDeviceProperties(&deviceProp, dev));
    printf("Using Device %d: %s\n", dev, deviceProp.name);
    CHECK(cudaSetDevice(dev));
}

//=============================================================================
// Device Helper Functions
//=============================================================================

/**
 * @brief GPU version: Compute diffuse lighting term
 */
__device__ double diffuse_device(const vec4 &point, const vec4 &normal, const Light &light) {
    vec4 ray_from_point_to_light = normalize(light.position - point);
    double cosTheta = dot(normal, ray_from_point_to_light);
    cosTheta = fmax(0.0, cosTheta);
    return cosTheta;
}

/**
 * @brief GPU version: Compute specular reflection term
 */
__device__ double reflection_device(const vec4 &point, const vec4 &normal,
                                     const vec4 &view, const Light &light) {
    if (diffuse_device(point, normal, light) > 0.0) {
        vec4 ray_from_point_to_light = normalize(light.position - point);
        vec4 ray_reflected = normalize(mirror(ray_from_point_to_light, normal));
        double cosTheta = dot(ray_reflected, view);
        cosTheta = fmax(0.0, cosTheta);
        return cosTheta;
    }
    return 0.0;
}

/**
 * @brief GPU version: Find intersection with scene
 */
__device__ bool intersect_scene_device(const Ray &ray, Material &intersection_material,
                                        vec4 &intersection_point, vec4 &intersection_normal,
                                        double &intersection_distance, const Data &data) {
    double tmin = DBL_MAX;

    // TODO: Implement BVH intersection for GPU
    // For now, this is a placeholder - you'll need to implement BVH traversal

    return (tmin < DBL_MAX);
}

/**
 * @brief GPU version: Forward declaration for trace_device (needed for recursion)
 */
__device__ vec4 trace_device(const Ray &ray,
                             int depth,
                             const Light *lights,
                             int numLights,
                             const Data &data,
                             const vec4 &background,
                             const vec4 &ambience,
                             int max_depth);

/**
 * @brief GPU version: Compute Phong lighting
 */
__device__ vec4 lighting_device(const vec4 &point,
                                const vec4 &normal,
                                const vec4 &view,
                                const Material &material,
                                const Light *lights,
                                int numLights,
                                const Data &data) {
    const double epsilon = 1e-4;
    vec4 color(0.0, 0.0, 0.0);

    // Ambient component
    color[0] += material.ambient[0];
    color[1] += material.ambient[1];
    color[2] += material.ambient[2];

    // Process each light source
    for (int i = 0; i < numLights; i++) {
        const Light &light = lights[i];

        // Compute diffuse and specular components
        double diffuse_ = diffuse_device(point, normal, light);
        double dot_rv = reflection_device(point, normal, view, light);
        double reflection_ = dot_rv;

        // Compute specular exponent
        for (double j = 0.0; j < material.shininess - 1; j += 1.0) {
            reflection_ *= dot_rv;
        }

        // Shadow calculation
        bool isShadow = false;
        if (material.shadowable) {
            Material shadow_material;
            vec4 shadow_point;
            vec4 shadow_normal;
            double shadow_t;
            vec4 light_direction = normalize(light.position - point);
            double light_distance = norm(light.position - point);
            Ray shadow_ray = Ray(point + epsilon * light_direction, light_direction);
            bool isIntersect = intersect_scene_device(shadow_ray, shadow_material,
                                                      shadow_point, shadow_normal,
                                                      shadow_t, data);
            isShadow = isIntersect && shadow_t < light_distance && 0.0 < shadow_t;
        }

        // Accumulate light contribution
        double shadow_factor = isShadow ? 0.0 : 1.0;
        color[0] += light.color[0] * shadow_factor * (material.diffuse[0] * diffuse_ + material.specular[0] * reflection_);
        color[1] += light.color[1] * shadow_factor * (material.diffuse[1] * diffuse_ + material.specular[1] * reflection_);
        color[2] += light.color[2] * shadow_factor * (material.diffuse[2] * diffuse_ + material.specular[2] * reflection_);
    }

    return color;
}

/**
 * @brief GPU version: Handle recursive ray tracing for reflections
 */
__device__ vec4 subtrace_device(const Ray &ray, const Material &material, const vec4 &point,
                                 const vec4 &normal, int depth, const Light *lights, int numLights,
                                 const Data &data, const vec4 &background, const vec4 &ambience,
                                 int max_depth) {
    if (material.mirror > 0.0) {
        vec4 v = reflect(ray.direction_, normal);
        const double epsilon = 1e-4;
        Ray reflected_ray = Ray(point + epsilon * v, v);
        return material.mirror * trace_device(reflected_ray, depth + 1, lights, numLights,
                                               data, background, ambience, max_depth);
    }
    return vec4(0.0, 0.0, 0.0);
}

/**
 * @brief GPU version: Main ray tracing function
 */
__device__ vec4 trace_device(const Ray &ray, int depth, const Light *lights, int numLights,
                             const Data &data, const vec4 &background, const vec4 &ambience,
                             int max_depth) {
    // Stop if recursion depth is too large
    if (depth > max_depth) {
        return vec4(0.0, 0.0, 0.0);
    }

    // Find intersection
    Material material;
    vec4 point;
    vec4 normal;
    double t;
    if (!intersect_scene_device(ray, material, point, normal, t, data)) {
        return background;
    }

    // Compute local Phong lighting
    vec4 color = (1.0 - material.mirror) * lighting_device(point, normal, -ray.direction_,
                                                             material, lights, numLights, data);

    // Compute global lighting (reflections)
    color += subtrace_device(ray, material, point, normal, depth, lights, numLights,
                            data, background, ambience, max_depth);

    return color;
}

//=============================================================================
// Kernel
//=============================================================================

__global__ void Raytracer::traceOnGPU(vec4 *pixels, int width, int height,
                                       Camera camera, Light *lights, int numLights,
                                       Data data, vec4 background, vec4 ambience,
                                       int max_depth) {
    // Calculate pixel coordinates
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    // Check bounds
    if (x >= width || y >= height) {
        return;
    }

    // Generate primary ray for this pixel
    Ray ray = camera.primary_ray(x, y);

    // Trace ray and get color
    vec4 color = trace_device(ray, 0, lights, numLights, data, background, ambience, max_depth);

    // Store result
    int pixelIdx = y * width + x;
    pixels[pixelIdx] = vec4(color[0], color[1], color[2], 1.0);
}
