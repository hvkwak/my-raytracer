// ============================================================================
// Solutions/Implementations by Hyovin Kwak to the course
// Computer Graphics @ TU Dortmund (Instructor: Prof. Dr. Mario Botsch)
//
// Note: The original exercise codebase is not included in this repo.
// ============================================================================

#ifdef CUDA_ENABLED
#include "common/common.h"
#include <cuda_runtime.h>
#include "mytracer_gpu.h"
#include "myutils_gpu.h"
#endif // CUDA_ENABLED

#include <stdio.h>
#include <ctime>
#include <cfloat>
#include <algorithm>

#include "utils/Material.h"
#include "utils/vec4.h"
#include "utils/Camera.h"
#include "utils/Ray.h"
#include "utils/Image.h"
#include "mydata.h"
#include "mybvh.h"

//=============================================================================
// functions from host
//=============================================================================

void init_device(void){
  // Initialize CUDA device
  int dev = 0;
  cudaDeviceProp deviceProp;
  CHECK(cudaGetDeviceProperties(&deviceProp, dev));
  printf("Using Device %d: %s\n", dev, deviceProp.name);
  CHECK(cudaSetDevice(dev));
}

/**
 * @brief launches compute_image_device kernel
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
                                 const BVH::BVHNodes_SoA* bvhNodes)
{
  int nThreads = 16;
  dim3 block(nThreads, nThreads);
  dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);

  // compute image
  printf("compute_image_device...\n");
  double iStart = seconds();
  compute_image_device<<<grid, block>>>(d_pixels,
                                        width,
                                        height,
                                        camera,
                                        d_lightsPos,
                                        d_lightsColor,
                                        nLights,
                                        background,
                                        ambience,
                                        max_depth,
                                        data,
                                        bvhNodes);
  CHECK(cudaDeviceSynchronize());
  CHECK(cudaGetLastError()) ;
  double iElaps = seconds() - iStart;
  printf("compute_image_device <<<Grid: %d, %d || Block: %d, %d >>>  Time elapsed %f sec.  \n", grid.x, grid.y, block.x, block.y, iElaps);

  const int subp = 4;
  const double threshold = 0.02;
  size_t nBytes = width*height*sizeof(vec4);
  CHECK(cudaMemcpy(d_tmpPixels, d_pixels, nBytes, cudaMemcpyDeviceToDevice));

  // do adaptive suersampling
  printf("adaptive_supersampling_device...\n");
  iStart = seconds();
  adaptive_supersampling_device<<<grid, block>>>(d_pixels,
                                                 d_tmpPixels,
                                                 width,
                                                 height,
                                                 camera,
                                                 d_lightsPos,
                                                 d_lightsColor,
                                                 nLights,
                                                 background,
                                                 ambience,
                                                 max_depth,
                                                 data,
                                                 bvhNodes,
                                                 subp,
                                                 threshold);
  CHECK(cudaDeviceSynchronize());
  CHECK(cudaGetLastError());
  iElaps = seconds() - iStart;
  printf("adaptive_supersampling <<<Grid: %d, %d || Block: %d, %d  >>>  Time elapsed %f sec.  \n", grid.x, grid.y, block.x, block.y, iElaps);

  // copy to host
  CHECK(cudaMemcpy(d_image, d_pixels, nBytes, cudaMemcpyDeviceToHost));
}

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

  // prevent over-saturation
  color = min(color, vec4(1, 1, 1));

  // Store result
  int pixelIdx = y * width + x;
  pixels[pixelIdx] = vec4(color[0], color[1], color[2]);
}

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
                                              const double threshold)
{
  // Calculate pixel coordinates
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  // Check bounds
  if (x >= width-1 || y >= height-1) {
    return;
  }
  if (x < 1 || y < 1) {
    return;
  }

  int pixelIdx = (y) * width + (x);

  // check if pixel color deviates to much from the surrounding
  vec4 color(0, 0, 0);
  vec4 c = tmpPixels[pixelIdx];
  double n = normSq(c - tmpPixels[(y) * width + (x+1)]) + normSq(c - tmpPixels[(y+1) * width + (x)]) +
             normSq(c - tmpPixels[(y) * width + (x-1)]) + normSq(c - tmpPixels[(y-1) * width + (x)]);

  if (n > threshold) {
    Ray ray;

    // shoot 16 rays through the pixel patch
    for (int si = 0; si < subp; si++) {
      const double xoffset =
          (si) / static_cast<double>(subp) - 0.5 + 1.0 / (2.0 * subp);
      for (int sj = 0; sj < subp; sj++) {
        const double yoffset =
            (sj) / static_cast<double>(subp) - 0.5 + 1.0 / (2.0 * subp);
        ray = camera.primary_ray(static_cast<double>(x) + xoffset,
                                  static_cast<double>(y) + yoffset);
        color += trace_device(ray,
                              background,
                              ambience,
                              max_depth,
                              lightsPos,
                              lightsColor,
                              nLights,
                              data,
                              bvhNodes);
      }
    }
    color /= subp * subp;

    // avoid over-saturation
    color = min(color, vec4(1, 1, 1));

    // store pixel color
    pixels[pixelIdx] = vec4(color[0], color[1], color[2]);
  }
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
                             const BVH::BVHNodes_SoA *bvhNodes)
{
  vec4 color(0.0, 0.0, 0.0);

  // Initial Intersection
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

  int depth = 0;
  double accumulated_weight = material.mirror;
  const double epsilon = 1e-4;
  vec4 v = reflect(ray.direction_, intersection_normal);
  while (depth < max_depth) {
    Ray reflected_ray = Ray(intersection_point + epsilon * v, v);
    // see if reflected ray intersects scene
    if (!intersect_scene_device(reflected_ray,
                                data,
                                bvhNodes,
                                material,
                                intersection_point,
                                intersection_normal,
                                intersection_distance))
    {
      color += accumulated_weight * background;
      break;
    }
    // update color. new intersection variables
    color += accumulated_weight * ((1.0 - material.mirror) * lighting_device(intersection_point,
                                                                             intersection_normal,
                                                                             -reflected_ray.direction_,
                                                                             material,
                                                                             data,
                                                                             bvhNodes,
                                                                             background,
                                                                             ambience,
                                                                             lightsPos,
                                                                             lightsColor,
                                                                             nLights));
    v = reflect(reflected_ray.direction_, intersection_normal);
    accumulated_weight *= material.mirror;
    depth++;
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
  }
  return (tmin < DBL_MAX);
}

/**
 * @brief Traverse BVH and find closest ray-triangle intersection (SoA version)
 * @param ray The ray to test
 * @param intersection_material Material at intersection point (output)
 * @param intersection_point Intersection point (output)
 * @param intersection_normal Normal at intersection (output)
 * @param intersection_distance Distance to intersection (input/output)
 * @param nodeIdx Current BVH node index
 * @return true if intersection found closer than intersection_distance
 */
__device__ bool intersectBVH_device(const Ray &ray,
                                    const Data *data,
                                    const BVH::BVHNodes_SoA *bvhNodes,
                                    Material &material,
                                    vec4 &intersection_point,
                                    vec4 &intersection_normal,
                                    double &intersection_distance) {

  int stack[64];
  int stackPtr = 0;

  stack[stackPtr++] = 0; // root nodeIdx is 0.
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
    if (!intersectAABB_device(ray, bb_min, bb_max, dummy)) {
      continue;
    }

    if (triCount > 0) {
      // printf("triCount: %d \n", triCount); // probably one triangle?
      // Leaf node - test all triangles (SoA data layout)
      double t;
      vec4 p, n, d;
      for (int i = firstTriIdx; i < firstTriIdx + triCount; i++) {

        // TODO: KEEPING THESE VARIABLES IS NO GOOD IDEA
        // take i, give it to intersect_triangle_device
        // let it handle them!

        // Fetch vertex indices
        int vi0 = data->vertexIdx_[i * 3];
        int vi1 = data->vertexIdx_[i * 3 + 1];
        int vi2 = data->vertexIdx_[i * 3 + 2];

        // Fetch vertex positions and normals
        vec4 vp0 = data->vertexPos_[vi0];
        vec4 vp1 = data->vertexPos_[vi1];
        vec4 vp2 = data->vertexPos_[vi2];
        vec4 normal = data->normals_[i];
        // check if triangle intersects.
        if (intersect_triangle_device(vp0, vp1, vp2,
                                      normal,
                                      vn0, vn1, vn2,
                                      ray,
                                      p, n, d, t, meshId, data))
        {
          if (t < intersection_distance)
          {
            // apply texture
            vec4 vn0 = data->vertexNormals_[vi0];
            vec4 vn1 = data->vertexNormals_[vi1];
            vec4 vn2 = data->vertexNormals_[vi2];

            // Fetch texture coordinates if available
            int meshId = data->vertexMeshId_[vi0];
            double u0 = 0, u1 = 0, u2 = 0;
            double v0 = 0, v1 = 0, v2 = 0;
            if (data->meshTexWidth_[meshId] != -1) { // hasTexture True
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

            intersect_triangle_texture_device();
            material.ambient = data->materialAmbient_[meshId];
            material.mirror = data->materialMirror_[meshId];
            material.shadowable = data->materialShadowable_[meshId];
            material.diffuse = d; // d is correct, because it can be from texture.
            material.specular = data->materialSpecular_[meshId];
            material.shininess = data->materialShininess_[meshId];
            intersection_point = p;
            intersection_normal = n;
            intersection_distance = t;
            hit = true;
          }
        }else{

        }


        // Intersect triangle
        if (intersect_triangle_device(vp0, vp1, vp2,
                                      normal,
                                      vn0, vn1, vn2,
                                      u0, u1, u2,
                                      v0, v1, v2,
                                      ray,
                                      p, n, d, t, meshId, data))
        {
          if (t < intersection_distance) {
            material.ambient = data->materialAmbient_[meshId];
            material.mirror = data->materialMirror_[meshId];
            material.shadowable = data->materialShadowable_[meshId];
            material.diffuse = d; // d is correct, because it can be from texture.
            material.specular = data->materialSpecular_[meshId];
            material.shininess = data->materialShininess_[meshId];
            intersection_point = p;
            intersection_normal = n;
            intersection_distance = t;
            hit = true;
          }
        }
      } // for loop ends here
    } else {
      double tminLeft, tminRight;

      // Cache node data once
      vec4 bb_min_left = bvhNodes->bb_min_[leftChildIdx];
      vec4 bb_max_left = bvhNodes->bb_max_[leftChildIdx];
      bool hitLeft = intersectAABB_device(ray, bb_min_left, bb_max_left, tminLeft);
      // bool hitLeft = intersectAABB(ray, bvhNodes_[node.leftChildIdx_].bb_min_, bvhNodes_[node.leftChildIdx_].bb_max_, tminLeft);

      vec4 bb_min_right = bvhNodes->bb_min_[leftChildIdx+1];
      vec4 bb_max_right = bvhNodes->bb_max_[leftChildIdx+1];
      bool hitRight = intersectAABB_device(ray, bb_min_right, bb_max_right, tminRight);
      // bool hitRight = intersectAABB(ray, bvhNodes_[node.leftChildIdx_+1].bb_min_, bvhNodes_[node.leftChildIdx_+1].bb_max_, tminRight);

      if (hitLeft && hitRight){
        if (tminLeft < tminRight){
          // Internal node - push children to stack: no recursion
          stack[stackPtr++] = leftChildIdx + 1; // right child
          stack[stackPtr++] = leftChildIdx; // left child will be visited first.
        }else{
          stack[stackPtr++] = leftChildIdx; // right child visited first
          stack[stackPtr++] = leftChildIdx+1; // left child
        }
      } else if (hitLeft) {
        stack[stackPtr++] = leftChildIdx; // left child will be visited first.
      } else if (hitRight) {
        stack[stackPtr++] = leftChildIdx+1; // left child will be visited first.
      }
    }
  }
  return hit;
}


/**
 * @brief Test ray-triangle intersection (SoA version)
 *
 * All triangle data is passed as parameters rather than stored in a Triangle struct
 * This version is optimized for GPU-friendly memory layouts
 *
 * @param p0,p1,p2 Triangle vertex positions
 * @param n Face normal
 * @param vn0,vn1,vn2 Vertex normals
 * @param u0,u1,u2,v0,v1,v2 Texture coordinates
 * @param ray Ray to test
 * @param intersection_point Intersection point (output)
 * @param intersection_normal Normal at intersection (output)
 * @param intersection_diffuse Diffuse color at intersection (output)
 * @param intersection_distance Distance to intersection (output)
 * @return true if ray intersects the triangle
 */

__device__ void intersect_triangle_texture_device(const vec4& n,
                                                  const vec4& vn0,
                                                  const vec4& vn1,
                                                  const vec4& vn2,
                                                  const double& u0,
                                                  const double& u1,
                                                  const double& u2,
                                                  const double& v0,
                                                  const double& v1,
                                                  const double& v2,
                                                  vec4 &intersection_diffuse,
                                                  const int & meshId,
                                                  const Data *data
{
  // Apply texture if available
  if (data->meshTexWidth_[meshId] != -1){
    // Interpolate texture coordinates with barycentric weights
    double u = alpha * u0 + beta * u1 + gamma * u2;
    double v = alpha * v0 + beta * v1 + gamma * v2;

    // Clamp to [0,1] range
    u = fmin(fmax(u, 0.0), 1.0);
    v = fmin(fmax(v, 0.0), 1.0);

    // Map to texture pixel coordinates
    const unsigned int W = data->meshTexWidth_[meshId];
    const unsigned int H = data->meshTexHeight_[meshId];
    int px = (int)round(u * (W - 1));
    int py = (int)round((1.0 - v) * (H - 1));
    size_t meshTexOffset = data->firstMeshTex_[meshId];
    vec4 texture = data->meshTexels_[meshTexOffset + py * size_t(W) + px]; // texture at (px, py)
    intersection_diffuse = texture;
  }
}

__device__ bool intersect_triangle_device(const vec4& p0,
                                          const vec4& p1,
                                          const vec4& p2,
                                          const vec4& n,
                                          const vec4& vn0,
                                          const vec4& vn1,
                                          const vec4& vn2,
                                          const Ray &ray,
                                          vec4 &intersection_point,
                                          vec4 &intersection_normal,
                                          vec4 &intersection_diffuse,
                                          double &intersection_distance,
                                          const int & meshId,
                                          const Data * data)
{
  intersection_diffuse = data->materialDiffuse_[meshId];

  // Solve for barycentric coordinates and ray parameter t
  // Same algorithm as AoS version
  const vec4 column1 = {p0[0] - p2[0], p0[1] - p2[1], p0[2] - p2[2]};
  const vec4 column2 = {p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]};
  const vec4 column3 = {-ray.direction_[0], -ray.direction_[1], -ray.direction_[2]};
  const vec4 column4 = {ray.origin_[0] - p2[0], ray.origin_[1] - p2[1], ray.origin_[2] - p2[2]};
  const double S = det4D_device(column1, column2, column3);

  // Check for degenerate triangle (determinant near zero)
  if (fabs(S) < 1e-10)
    return false;

  const double alpha = det4D_device(column4, column2, column3) / S;
  const double beta = det4D_device(column1, column4, column3) / S;
  const double gamma = (1.0 - alpha - beta);
  const double eps_shadow_acne = 1e-5;

  // check if t is correct: positive && beyond shadow acne
  const double t = det4D_device(column1, column2, column4) / S;
  if (t <= eps_shadow_acne)
    return false;

  // check if it's inside
  bool isInside = true; // shadow acne guard
  isInside = isInside && 0.0 <= alpha && alpha <= 1.0;
  isInside = isInside && 0.0 <= beta && beta <= 1.0;
  isInside = isInside && 0.0 <= gamma && gamma <= 1.0;
  if (!isInside)
    return false;

  // save intersection parameters
  intersection_distance = t;
  intersection_point = ray(t);

  // Compute normal (flat or interpolated)
  if (data->meshDrawMode_[meshId] == 0) {
    // FLAT
    // intersection_normal = normalize(cross(p1 - p0, p2 - p0));
    intersection_normal = n;
  } else if(data->meshDrawMode_[meshId] == 1) {
    // Phong shading
    intersection_normal = alpha * vn0 + beta * vn1 + gamma * vn2;
  }else{
    printf("Invalid meshDrawMode_[meshId], meshId: %d, its shading mode: %d", meshId, data->meshDrawMode_[meshId]);
  }
  return true;
}

__device__ bool intersectAABB_device(const Ray & ray, const vec4 & bb_min_, const vec4 & bb_max_, double & tmin_){

  // TODO: check division with zero due to ray.direction_?

  // Slab method for ray-AABB intersection
  double tmin = (bb_min_[0] - ray.origin_[0]) / ray.direction_[0];
  double tmax = (bb_max_[0] - ray.origin_[0]) / ray.direction_[0];

  if (tmin > tmax) {
    double temp = tmin;
    tmin = tmax;
    tmax = temp;
  }

  double tymin = (bb_min_[1] - ray.origin_[1]) / ray.direction_[1];
  double tymax = (bb_max_[1] - ray.origin_[1]) / ray.direction_[1];

  if (tymin > tymax) {
    double temp = tymin;
    tymin = tymax;
    tymax = temp;
  }

  if ((tmin > tymax) || (tymin > tmax))
    return false;

  tmin = fmax(tmin, tymin);
  tmax = fmin(tmax, tymax);

  double tzmin = (bb_min_[2] - ray.origin_[2]) / ray.direction_[2];
  double tzmax = (bb_max_[2] - ray.origin_[2]) / ray.direction_[2];

  if (tzmin > tzmax) {
    double temp = tzmin;
    tzmin = tzmax;
    tzmax = temp;
  }

  if ((tmin > tzmax) || (tzmin > tmax))
    return false;

  tmin = fmax(tmin, tzmin);
  tmax = fmin(tmax, tzmax);

  // TODO: tmin_ may be useful for left, right child check
  tmin_ = tmin;

  return tmax > 1e-5;  // Cull if farther than current closest hit
}



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
    reflection_ = pow(reflection_, material.shininess);
    // double i = 0.0;
    // while (i < material.shininess - 1){
    //   reflection_ *= dot_rv;
    //   i += 1.0;
    // }

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
