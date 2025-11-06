// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#ifndef MYBVH_H
#define MYBVH_H

#include "utils/vec3.h"
#include "utils/Ray.h"
#include "utils/Material.h"
#include "utils/Triangle.h"
#include "mydata.h"
#include "Mesh.h"
#include <math.h>
#include <cfloat>

/**
 * @brief Bounding Volume Hierarchy for accelerated ray-triangle intersection
 *
 * Provides two variants:
 * - Standard AoS (Array of Structures) version using Triangle objects
 * - SoA (Structure of Arrays) version for GPU-friendly memory layout
 */
class BVH {

public:

  BVH() = default;
  ~BVH();

  /**
   * @brief Initialize BVH with standard AoS layout
   * @param meshes Vector of mesh pointers to build BVH from
   */
  void init(std::vector<Mesh*> &meshes);

  /**
   * @brief Initialize BVH with SoA layout for GPU acceleration
   * @param meshes Vector of mesh pointers
   * @param data Pointer to SoA data structure
   */
  void initSoA(std::vector<Mesh*> &meshes, Data* data);

  /**
   * @brief Check if BVH has been initialized
   * @return true if initialized
   */
  bool isInitialized() const;

  /**
   * @brief Test ray intersection with axis-aligned bounding box
   * @param ray The ray to test
   * @param bb_min_ Minimum corner of AABB
   * @param bb_max_ Maximum corner of AABB
   * @return true if ray intersects the AABB and intersection is closer than max_distance
   */
  bool intersectAABB(const Ray & ray, const vec3 & bb_min_, const vec3 & bb_max_, double & tmin_) const;

  /**
   * @brief Traverse BVH and find closest ray-triangle intersection (AoS version)
   * @param ray The ray to test
   * @param intersection_material Material at intersection point (output)
   * @param intersection_point Intersection point (output)
   * @param intersection_normal Normal at intersection (output)
   * @param intersection_distance Distance to intersection (input/output)
   * @param nodeIdx Current BVH node index
   * @return true if intersection found closer than intersection_distance
   */
  bool intersectBVH(const Ray &ray,
                    Material& intersection_material,
                    vec3 &intersection_point,
                    vec3 &intersection_normal,
                    double &intersection_distance,
                    int nodeIdx) const;

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
  bool intersectBVHSoA(const Ray &ray,
                        Material& intersection_material,
                        vec3 &intersection_point,
                        vec3 &intersection_normal,
                        double &intersection_distance) const;

private:

  /**
   * @brief BVH node structure for AoS
   *
   * Internal nodes have triCount == 0 and store leftChildIdx (right is leftChildIdx + 1)
   * Leaf nodes have triCount > 0 and store firstTriIdx to triangle data
   */
  struct BVHNode {
    vec3 bb_min_, bb_max_;  ///< Axis-aligned bounding box
    int leftChildIdx_;       ///< Left child index (right child is leftChildIdx + 1)
    int firstTriIdx_;        ///< First triangle index (for leaf nodes)
    int triCount_;           ///< Number of triangles (0 for internal nodes)
  };



  // ===== AoS-specific methods =====

  /**
   * @brief Extract triangle data from meshes into flat array
   */
  void getData(std::vector<Mesh*> &meshes);

  /**
   * @brief Compute bounding box for a node (AoS version)
   */
  void updateNodeBounds(int nodeIdx);

  /**
   * @brief Recursively subdivide BVH node (AoS version)
   */
  void subdivide(int nodeIdx, int depth);

  /**
   * @brief Partition triangles around split position (AoS version)
   */
  void inplace_partition(int nodeIdx, double splitPos, int axis, int& i);

  // ===== SoA-specific methods =====

  /**
   * @brief Compute bounding box for a node (SoA version)
   */
  void updateNodeBoundsSoA(int nodeIdx);

  /**
   * @brief Recursively subdivide BVH node (SoA version)
   */
  void subdivideSoA(int nodeIdx, int depth);

  /**
   * @brief Partition triangles around split position (SoA version)
   */
  void inplace_partitionSoA(int nodeIdx, double splitPos, int axis, int & i);

  // ===== Splitting heuristics =====

  /**
   * @brief Compute median centroid position for splitting (AoS version)
   */
  double median(int axis, int nodeIdx);

  /**
   * @brief Compute median centroid position for splitting (SoA version)
   */
  double medianSoA(int axis, int nodeIdx);

  /**
   * @brief Compute median of array in-place
   */
  double median_inplace(std::vector<double> &a);

  // ===== Data members =====

  std::vector<BVHNode> bvhNodes_;      ///< BVH node pool for AoS
  std::vector<Triangle*> triangles_;   ///< Triangle pointers (AoS version)
  std::vector<Mesh*> meshes_;          ///< Mesh pointers

  int triCount = 0;                    ///< Total triangle count
  int rootNodeIdx_ = 0;                ///< Root node index
  int nodesUsed_ = 1;                  ///< Number of nodes allocated
  bool isInitialized_ = false;         ///< Initialization flag

  Data* data_;                         ///< SoA data structure pointer

  // BVHNode attributes in SoA
  // struct BVHNodes_SoA {
  //   std::vector<vec3> bb_min_;
  //   std::vector<vec3> bb_max_;
  //   std::vector<int> leftChildIdx_;
  //   std::vector<int> firstTriIdx_;
  //   std::vector<int> triCount_;
  // };
  // BVHNodes_SoA bvhNodesSoA_;
};

// ===== Utility Functions =====================================================

/**
 * @brief Component-wise minimum of two vec3
 */
inline vec3 fmin(const vec3 &a, const vec3 &b) {
  return vec3(fmin(a[0], b[0]), fmin(a[1], b[1]), fmin(a[2], b[2]));
}

/**
 * @brief Component-wise maximum of two vec3
 */
inline vec3 fmax(const vec3 &a, const vec3 &b) {
  return vec3(fmax(a[0], b[0]), fmax(a[1], b[1]), fmax(a[2], b[2]));
}

#endif // MYBVH_H
