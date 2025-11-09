// ============================================================================
// Computer Graphics(Graphische Datenverarbeitung) - TU Dortmund
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
// #include "Raytracer.h"
#include "utils/Material.h"
// #include "utils/Object.h"
#include "utils/vec4.h"
#include "utils/Camera.h"
#include "utils/Ray.h"
#include "mytracer_gpu.h"
#include "mydata.h"
#include "mybvh.h"

//=============================================================================
// Kernel
//=============================================================================

__global__ void compute_image_device(vec4 *pixels,
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
                                     const BVH::BVHNodes_SoA *bvhNodes) {
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
  vec4 color = trace_device(ray,
                            background,
                            ambience,
                            max_depth,
                            lightsPos,
                            lightsColor,
                            nLights,
                            data,
                            bvhNodes);

  // // Store result
  int pixelIdx = y * width + x;
  pixels[pixelIdx] = vec4(color[0], color[1], color[2]);
}

//=============================================================================
// Device Functions
//=============================================================================

/**
 * @brief GPU version of trace();
 */
__device__ vec4 trace_device(const Ray &ray,
                             const vec4 &background,
                             const vec4 &ambience,
                             const int &max_depth,
                             const vec4* lightsPos,
                             const vec4* lightsColor,
                             const int nLights,
                             const Data *data,
                             const BVH::BVHNodes_SoA *bvhNodes){

  vec4 color(0.0, 0.0, 0.0);
  if (max_depth < 1) {
    return color;
  }

  // Intersection results
  Material material;
  vec4 intersection_point;
  vec4 intersection_normal;
  double intersection_distance;

  if (!intersect_scene_device(ray,
                              data,
                              bvhNodes,
                              material,
                              intersection_point,
                              intersection_normal,
                              intersection_distance))
  {
    return background;
  }

  color += (1.0 - material.mirror) * lighting_device(intersection_point,
                                                     intersection_normal,
                                                     -ray.direction_,
                                                     material,
                                                     data,
                                                     bvhNodes,
                                                     background,
                                                     ambience,
                                                     lightsPos,
                                                     lightsColor,
                                                     nLights);

  int i = 1;
  const double epsilon = 1e-4;
  while (i < max_depth) {
    if (material.mirror > 0.0) {
      vec4 v = reflect(ray.direction_, intersection_normal);
      Ray reflected_ray = Ray(intersection_point + epsilon * v, v);
      //
      // TODO: intersect_scene takes place here + early stopping
      //
      color += material.mirror * ((1.0 - material.mirror) * lighting_device(intersection_point,
                                                                            intersection_normal,
                                                                            -ray.direction_,
                                                                            material,
                                                                            data,
                                                                            bvhNodes,
                                                                            background,
                                                                            ambience,
                                                                            lightsPos,
                                                                            lightsColor,
                                                                            nLights));
    }
    i++;
  }
  return color;
}

__device__ bool intersect_scene_device(const Ray & ray,
                                       const Data *data,
                                       const BVH::BVHNodes_SoA *bvhNodes,
                                       Material &material,
                                       vec4 &intersection_point,
                                       vec4 &intersection_normal,
                                       double &intersection_distance)
{
  double tmin(DBL_MAX);

  if (intersectBVH_device(ray, data, bvhNodes, material, intersection_point, intersection_normal, tmin)) {
    intersection_distance = tmin;
    return true;
  }
  return false;
}

__device__ bool intersectBVH_device(const Ray &ray,
                                    const Data *data,
                                    const BVH::BVHNodes_SoA *bvhNodes,
                                    Material &material,
                                    vec4 &intersection_point,
                                    vec4 &intersection_normal,
                                    double &intersection_distance) {

  const int rootNodeIdx = 0;
  int stack[64];
  int stackPtr = 0;

  stack[stackPtr++] = rootNodeIdx;
  bool hit = false;

  while (stackPtr > 0){

    // Pop node from stack
    int nodeIdx = stack[--stackPtr];

    // Cache node data once
    vec4 bb_min = bvhNodes->bb_min_[nodeIdx];
    vec4 bb_max = bvhNodes->bb_max_[nodeIdx];
    int triCount = bvhNodes->triCount_[nodeIdx];
    int firstTriIdx = bvhNodes->firstTriIdx_[nodeIdx];
    int leftChildIdx = bvhNodes->leftChildIdx_[nodeIdx];
    // const BVHNode& node = bvhNodes[nodeIdx];
    double dummy;

    // Early exit if ray doesn't intersect node's bounding box
    // if (!intersectAABB(ray, node.bb_min_, node.bb_max_, dummy)) {
    if (!intersectAABB(ray, bb_min, bb_max, dummy)) {
      continue;
    }

    if (triCount > 0) {
      // Leaf node - test all triangles (SoA data layout)
      double t;
      vec4 p, n, d;
      for (int i = firstTriIdx; i < firstTriIdx + triCount; i++) {
        // Fetch vertex indices
        int vi0 = data->vertexIdx_[i * 3];
        int vi1 = data->vertexIdx_[i * 3 + 1];
        int vi2 = data->vertexIdx_[i * 3 + 2];

        // Fetch vertex positions and normals
        vec4 vp0 = data->vertexPos_[vi0];
        vec4 vp1 = data->vertexPos_[vi1];
        vec4 vp2 = data->vertexPos_[vi2];
        vec4 normal = data->normals_[i];
        vec4 vn0 = data->vertexNormals_[vi0];
        vec4 vn1 = data->vertexNormals_[vi1];
        vec4 vn2 = data->vertexNormals_[vi2];

        Mesh *mesh = data->meshes_[vi0];
        int meshId = data->vertexMeshId_[vi0]; // TODO: use this!


        // Fetch texture coordinates if available
        double u0 = 0, u1 = 0, u2 = 0;
        double v0 = 0, v1 = 0, v2 = 0;
        if (mesh->hasTexture_) {
          int iuv0 = data->textureIdx_[i * 3];
          int iuv1 = data->textureIdx_[i * 3 + 1];
          int iuv2 = data->textureIdx_[i * 3 + 2];
          u0 = data->textureCoordinatesU_[iuv0];
          u1 = data->textureCoordinatesU_[iuv1];
          u2 = data->textureCoordinatesU_[iuv2];
          v0 = data->textureCoordinatesV_[iuv0];
          v1 = data->textureCoordinatesV_[iuv1];
          v2 = data->textureCoordinatesV_[iuv2];
        }

        // Intersect triangle
        if (mesh->intersect_triangle_SoA(vp0, vp1, vp2, normal, vn0, vn1, vn2,
                                         u0, u1, u2, v0, v1, v2, ray, p, n, d,
                                         t)) {
          if (t < intersection_distance) {
            material = mesh->material_;
            material.diffuse = d;
            intersection_point = p;
            intersection_normal = n;
            intersection_distance = t;
            hit = true;
          }
        }
      }
    } else {
      double tminLeft, tminRight;

      // Cache node data once
      // vec4 bb_min_left = d_bvhNodesSoA_->bb_min_[leftChildIdx];
      // vec4 bb_max_left = d_bvhNodesSoA_->bb_max_[leftChildIdx];
      // bool hitLeft = intersectAABB(ray, bb_min_left, bb_max_left, tminLeft);
      bool hitLeft = intersectAABB(ray, bvhNodes_[node.leftChildIdx_].bb_min_, bvhNodes_[node.leftChildIdx_].bb_max_, tminLeft);

      // vec4 bb_min_right = d_bvhNodesSoA_->bb_min_[leftChildIdx+1];
      // vec4 bb_max_right = d_bvhNodesSoA_->bb_max_[leftChildIdx+1];
      // bool hitRight = intersectAABB(ray, bb_min_right, bb_max_right, tminRight);
      bool hitRight = intersectAABB(ray, bvhNodes_[node.leftChildIdx_+1].bb_min_, bvhNodes_[node.leftChildIdx_+1].bb_max_, tminRight);

      if (hitLeft && hitRight){
        if (tminLeft < tminRight){
          // Internal node - push children to stack: no recursion
          stack[stackPtr++] = node.leftChildIdx_ + 1; // right child
          stack[stackPtr++] = node.leftChildIdx_; // left child will be visited first.
        }else{
          stack[stackPtr++] = node.leftChildIdx_; // right child visited first
          stack[stackPtr++] = node.leftChildIdx_+1; // left child
        }
      } else if (hitLeft) {
        stack[stackPtr++] = node.leftChildIdx_; // left child will be visited first.
      } else if (hitRight) {
        stack[stackPtr++] = node.leftChildIdx_+1; // left child will be visited first.
      }


    }
  }
  return hit;
}


/**
 * @brief GPU version: Compute diffuse lighting term
 */
__device__ double diffuse_device(const vec4 &point, const vec4 &normal, const vec4 &lightPos) {
    vec4 ray_from_point_to_light = normalize(lightPos - point);
    double cosTheta = dot(normal, ray_from_point_to_light);
    cosTheta = fmax(0.0, cosTheta);
    return cosTheta;
}

/**
 * @brief GPU version: Compute specular reflection term
 */
__device__ double reflection_device(const vec4 &point, const vec4 &normal, const vec4 &view, const vec4 &lightPos) {
    if (diffuse_device(point, normal, lightPos) > 0.0) {
        vec4 ray_from_point_to_light = normalize(lightPos - point);
        vec4 ray_reflected = normalize(mirror(ray_from_point_to_light, normal));
        double cosTheta = dot(ray_reflected, view);
        cosTheta = fmax(0.0, cosTheta);
        return cosTheta;
    }
    return 0.0;
}

// /**
//  * @brief GPU version: Find intersection with scene
//  */
// __device__ bool intersect_scene_device(const Ray &ray, Material &intersection_material,
//                                         vec4 &intersection_point, vec4 &intersection_normal,
//                                         double &intersection_distance, const Data &data) {
//     double tmin = DBL_MAX;

//     // TODO: Implement BVH intersection for GPU
//     // For now, this is a placeholder - you'll need to implement BVH traversal

//     return (tmin < DBL_MAX);
// }



/**
 * @brief Compute Phong lighting model (ambient + diffuse + specular)
 *
 * Computes local illumination at a surface point using the Phong shading model
 *
 * @param point Intersection point
 * @param normal Surface normal at intersection
 * @param view Direction from intersection point to viewer
 * @param material Material properties
 * @return Total color contribution from all light sources
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
                                const int nLights)
{
  const double epsilon = 1e-4;  // Offset to avoid self-intersection
  vec4 color(0.0, 0.0, 0.0);

  // Ambient component (approximates global illumination)
  color[0] += ambience[0] * material.ambient[0];
  color[1] += ambience[1] * material.ambient[1];
  color[2] += ambience[2] * material.ambient[2];

  // Process each light source
  for (int k = 0; k < nLights; k++) {
    vec4 light_position = lightsPos[k];
    vec4 light_color = lightsColor[k];
    // Compute diffuse and specular components
    double diffuse_ = diffuse_device(point, normal, light_position);
    double dot_rv = reflection_device(point, normal, view, light_position);
    double reflection_ = dot_rv;
    // Compute specular exponent
    double i = 0.0;
    while (i < material.shininess - 1){
      reflection_ *= dot_rv;
      i += 1.0;
    }

    // Shadow calculation
    bool isShadow = false;
    if (material.shadowable){
      Material shadow_material;
      vec4 shadow_point;
      vec4 shadow_normal;
      double shadow_t;
      vec4 light_direction = normalize(light_position - point);
      double light_distance = norm(light_position - point);
      Ray shadow_ray = Ray(point + epsilon * light_direction, light_direction);
      bool isIntersect = intersect_scene_device(shadow_ray,
                                                data,
                                                bvhNodes,
                                                shadow_material,
                                                shadow_point,
                                                shadow_normal,
                                                shadow_t);
      isShadow = isIntersect && shadow_t < light_distance && 0.0 < shadow_t;
    }

    // Accumulate light contribution if not in shadow
    color[0] += light_color[0] * !isShadow * (material.diffuse[0] * diffuse_ + material.specular[0] * reflection_);
    color[1] += light_color[1] * !isShadow * (material.diffuse[1] * diffuse_ + material.specular[1] * reflection_);
    color[2] += light_color[2] * !isShadow * (material.diffuse[2] * diffuse_ + material.specular[2] * reflection_);
  }
  return color;
}



// /**
//  * @brief GPU version: Handle recursive ray tracing for reflections
//  */
// __device__ vec4 subtrace_device(const Ray &ray, const Material &material, const vec4 &point,
//                                  const vec4 &normal, int depth, const Light *lights, int numLights,
//                                  const Data &data, const vec4 &background, const vec4 &ambience,
//                                  int max_depth) {
//     if (material.mirror > 0.0) {
//         vec4 v = reflect(ray.direction_, normal);
//         const double epsilon = 1e-4;
//         Ray reflected_ray = Ray(point + epsilon * v, v);
//         return material.mirror * trace_device(reflected_ray, depth + 1, lights, numLights,
//                                                data, background, ambience, max_depth);
//     }
//     return vec4(0.0, 0.0, 0.0);
// }

// /**
//  * @brief GPU version: Main ray tracing function
//  */
// __device__ vec4 trace_device(const Ray &ray, int depth, const Light *lights, int numLights,
//                              const Data &data, const vec4 &background, const vec4 &ambience,
//                              int max_depth) {
//     // Stop if recursion depth is too large
//     if (depth > max_depth) {
//         return vec4(0.0, 0.0, 0.0);
//     }

//     // Find intersection
//     Material material;
//     vec4 point;
//     vec4 normal;
//     double t;
//     if (!intersect_scene_device(ray, material, point, normal, t, data)) {
//         return background;
//     }

//     // Compute local Phong lighting
//     vec4 color = (1.0 - material.mirror) * lighting_device(point, normal, -ray.direction_,
//                                                              material, lights, numLights, data);

//     // Compute global lighting (reflections)
//     color += subtrace_device(ray, material, point, normal, depth, lights, numLights,
//                             data, background, ambience, max_depth);

//     return color;
// }

