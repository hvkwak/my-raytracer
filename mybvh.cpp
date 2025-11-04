// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#include "mybvh.h"
#include <algorithm>
#include <cfloat>
#include <math.h>
#include <vector>

// =============================================================================
// Public Methods
// =============================================================================
BVH::~BVH(){}

void BVH::init(std::vector<Mesh*> &meshes) {

  if (meshes.size() > 0){
    // Extract triangle data from meshes
    getData(meshes);

    // Allocate node pool
    bvhNodes_.resize(2 * triCount - 1);

    // Initialize root node
    BVHNode &root = bvhNodes_[rootNodeIdx_];
    root.leftChildIdx = 0;
    root.firstTriIdx = 0;
    root.triCount = triCount;
    updateNodeBounds(rootNodeIdx_);

    // Build BVH recursively
    subdivide(rootNodeIdx_, 1);

    isInitialized_ = true;

    // Debug: print first 50 nodes
    for (int i = 0; i < 50 && i < bvhNodes_.size(); i++) {
      BVHNode node = bvhNodes_.at(i);
      std::cout << i << "-th Node"
                << " bb_min_: " << node.bb_min_
                << " bb_max_: " << node.bb_max_
                << " leftChildIdx: " << node.leftChildIdx
                << " firstTriIdx: " << node.firstTriIdx
                << " triCount: " << node.triCount << std::endl;
    }
  } else {
    std::cout << "No mesh available. BVH build stopped." << std::endl;
  }
}

void BVH::initSoA(std::vector<Mesh*> &meshes, Data* data)
{
  if (meshes.size() > 0){
    meshes_ = meshes;
    data_ = data;
    triCount = data_->vertexIdx_.size() / 3;

    // Allocate node pool
    bvhNodes_.resize(2 * triCount - 1);

    // Initialize root node
    BVHNode &root = bvhNodes_[rootNodeIdx_];
    root.leftChildIdx = 0;
    root.firstTriIdx = 0;
    root.triCount = triCount;
    updateNodeBoundsSoA(rootNodeIdx_);

    // Build BVH recursively
    subdivideSoA(rootNodeIdx_, 1);

    isInitialized_ = true;

    // Debug: print first 50 nodes
    for (int i = 0; i < 50 && i < bvhNodes_.size(); i++) {
      BVHNode node = bvhNodes_.at(i);
      std::cout << i << "-th Node"
                << " bb_min_: " << node.bb_min_
                << " bb_max_: " << node.bb_max_
                << " leftChildIdx: " << node.leftChildIdx
                << " firstTriIdx: " << node.firstTriIdx
                << " triCount: " << node.triCount << std::endl;
    }
  } else {
    std::cout << "No mesh available. BVH build stopped." << std::endl;
  }
}

bool BVH::isInitialized() const{
  return isInitialized_;
}

bool BVH::intersectAABB(const Ray & ray, const vec3 & bb_min_, const vec3 & bb_max_) const{
  // Slab method for ray-AABB intersection
  double tmin = (bb_min_[0] - ray.origin_[0]) / ray.direction_[0];
  double tmax = (bb_max_[0] - ray.origin_[0]) / ray.direction_[0];

  if (tmin > tmax)
    std::swap(tmin, tmax);

  double tymin = (bb_min_[1] - ray.origin_[1]) / ray.direction_[1];
  double tymax = (bb_max_[1] - ray.origin_[1]) / ray.direction_[1];

  if (tymin > tymax)
    std::swap(tymin, tymax);

  if ((tmin > tymax) || (tymin > tmax))
    return false;

  tmin = std::max(tmin, tymin);
  tmax = std::min(tmax, tymax);

  double tzmin = (bb_min_[2] - ray.origin_[2]) / ray.direction_[2];
  double tzmax = (bb_max_[2] - ray.origin_[2]) / ray.direction_[2];

  if (tzmin > tzmax)
    std::swap(tzmin, tzmax);

  if ((tmin > tzmax) || (tzmin > tmax))
    return false;

  tmax = std::min(tmax, tzmax);

  return tmax > 1e-5;  // Ensure intersection is in front of ray
}


bool BVH::intersectBVH(const Ray& ray,
                       Material& intersection_material,
                       vec3 &intersection_point,
                       vec3 &intersection_normal,
                       double &intersection_distance,
                       int nodeIdx) const{
  const BVHNode & node = bvhNodes_.at(nodeIdx);

  // Early exit if ray doesn't intersect node's bounding box
  if (!intersectAABB(ray, node.bb_min_, node.bb_max_)){
    return false;
  }

  if (node.triCount > 0) {
    // Leaf node - test all triangles
    double t;
    vec3 p, n, d;
    for (int i = node.firstTriIdx; i < node.firstTriIdx + node.triCount; i++){
      Triangle* tri = triangles_.at(i);
      Mesh* mesh = meshes_.at(tri->meshIdx);
      if (mesh->intersect_triangle(*tri, ray, p, n, d, t)) {
        if (t < intersection_distance) {
          intersection_material = mesh->material_;
          intersection_material.diffuse = d;
          intersection_point = p;
          intersection_normal = n;
          intersection_distance = t;
        }
      }
    }
    return (intersection_distance < DBL_MAX);
  } else {
    // Internal node - recursively test both children
    bool isLeft = intersectBVH(ray,intersection_material,intersection_point,intersection_normal, intersection_distance, node.leftChildIdx);
    bool isRight = intersectBVH(ray,intersection_material,intersection_point,intersection_normal, intersection_distance, node.leftChildIdx + 1);
    return isLeft || isRight;
  }
}

bool BVH::intersectBVHSoA(const Ray &ray,
                           Material& intersection_material,
                           vec3 &intersection_point,
                           vec3 &intersection_normal,
                           double &intersection_distance,
                           int nodeIdx) const {
  const BVHNode & node = bvhNodes_.at(nodeIdx);

  // Early exit if ray doesn't intersect node's bounding box
  if (!intersectAABB(ray, node.bb_min_, node.bb_max_)){
    return false;
  }

  if (node.triCount > 0) {
    // Leaf node - test all triangles (SoA data layout)
    double t;
    vec3 p, n, d;
    for (int i = node.firstTriIdx; i < node.firstTriIdx + node.triCount; i++){
      // Fetch vertex indices
      int vi0 = data_->vertexIdx_[i*3];
      int vi1 = data_->vertexIdx_[i*3+1];
      int vi2 = data_->vertexIdx_[i*3+2];

      // Fetch vertex positions and normals
      vec3 vp0 = data_->vertexPos_[vi0];
      vec3 vp1 = data_->vertexPos_[vi1];
      vec3 vp2 = data_->vertexPos_[vi2];
      vec3 normal = data_->normals_[i];
      vec3 vn0 = data_->vertexNormals_[vi0];
      vec3 vn1 = data_->vertexNormals_[vi1];
      vec3 vn2 = data_->vertexNormals_[vi2];

      Mesh* mesh = data_->meshes_[vi0];

      // Fetch texture coordinates if available
      double u0 = 0, u1 = 0, u2 = 0;
      double v0 = 0, v1 = 0, v2 = 0;
      if (mesh->hasTexture_){
        int iuv0 = data_->textureIdx_[i*3];
        int iuv1 = data_->textureIdx_[i*3+1];
        int iuv2 = data_->textureIdx_[i*3+2];
        u0 = data_->textureCoordinatesU_[iuv0];
        u1 = data_->textureCoordinatesU_[iuv1];
        u2 = data_->textureCoordinatesU_[iuv2];
        v0 = data_->textureCoordinatesV_[iuv0];
        v1 = data_->textureCoordinatesV_[iuv1];
        v2 = data_->textureCoordinatesV_[iuv2];
      }

      // Intersect triangle
      if (mesh->intersect_triangle_SoA(vp0, vp1, vp2, normal, vn0, vn1, vn2,
                                        u0, u1, u2, v0, v1, v2,
                                        ray, p, n, d, t)) {
        if (t < intersection_distance) {
          intersection_material = mesh->material_;
          intersection_material.diffuse = d;
          intersection_point = p;
          intersection_normal = n;
          intersection_distance = t;
        }
      }
    }
    return (intersection_distance < DBL_MAX);
  } else {
    // Internal node - recursively test both children
    bool isLeft = intersectBVHSoA(ray,intersection_material,intersection_point,intersection_normal, intersection_distance, node.leftChildIdx);
    bool isRight = intersectBVHSoA(ray,intersection_material,intersection_point,intersection_normal, intersection_distance, node.leftChildIdx + 1);
    return isLeft || isRight;
  }
}
// =============================================================================
// Private Methods - AoS Version
// =============================================================================

void BVH::getData(std::vector<Mesh*> &meshes){
  meshes_ = meshes;
  int i = 0;
  for (Mesh* mesh : meshes_){
    for (Triangle &tri : mesh->triangles_) {
      // Compute triangle centroid for splitting
      vec3 vertex0 = mesh->vertices_.at(tri.i0).position;
      vec3 vertex1 = mesh->vertices_.at(tri.i1).position;
      vec3 vertex2 = mesh->vertices_.at(tri.i2).position;
      vec3 centroid = (vertex0 + vertex1 + vertex2) / 3.0;
      tri.centroid = centroid;
      tri.meshIdx = i;
      triangles_.push_back(&tri);
    }
    triCount = triCount + mesh->triangles_.size();
    i++;
  }
}

void BVH::updateNodeBounds(int nodeIdx){
  BVHNode& node = bvhNodes_.at(nodeIdx);
  node.bb_min_ = vec3(DBL_MAX);
  node.bb_max_ = vec3(-DBL_MAX);

  // Expand bounding box to contain all triangle vertices
  for (int i = node.firstTriIdx; i < node.firstTriIdx + node.triCount; i++){
    Triangle* tri = triangles_.at(i);
    Mesh* mesh = meshes_.at(tri->meshIdx);
    node.bb_min_ = fmin(node.bb_min_, mesh->vertices_.at(tri->i0).position);
    node.bb_min_ = fmin(node.bb_min_, mesh->vertices_.at(tri->i1).position);
    node.bb_min_ = fmin(node.bb_min_, mesh->vertices_.at(tri->i2).position);
    node.bb_max_ = fmax(node.bb_max_, mesh->vertices_.at(tri->i0).position);
    node.bb_max_ = fmax(node.bb_max_, mesh->vertices_.at(tri->i1).position);
    node.bb_max_ = fmax(node.bb_max_, mesh->vertices_.at(tri->i2).position);
  }
}


void BVH::subdivide(int nodeIdx, int depth){
  BVHNode& node = bvhNodes_.at(nodeIdx);

  // Terminate recursion if node has few triangles
  if (node.triCount <= 2) return;

  // Determine split axis (cycle through x, y, z based on depth)
  int axis = depth % 3;
  double splitPos = median(axis, nodeIdx);

  // Partition triangles around split position
  int i = node.firstTriIdx;
  inplace_partition(nodeIdx, splitPos, axis, i);

  // Abort split if one side is empty
  int leftCount = i - node.firstTriIdx;
  if (leftCount == 0 || leftCount == node.triCount) return;

  // Create child nodes
  int leftChildIdx = nodesUsed_;
  int rightChildIdx = leftChildIdx + 1;
  nodesUsed_ = nodesUsed_ + 2;
  bvhNodes_[leftChildIdx].firstTriIdx = node.firstTriIdx;
  bvhNodes_[leftChildIdx].triCount = leftCount;
  bvhNodes_[rightChildIdx].firstTriIdx = i;
  bvhNodes_[rightChildIdx].triCount = node.triCount - leftCount;
  node.leftChildIdx = leftChildIdx;
  node.triCount = 0;
  updateNodeBounds(leftChildIdx);
  updateNodeBounds(rightChildIdx);

  // Recursively subdivide children
  subdivide(leftChildIdx, depth + 1);
  subdivide(rightChildIdx, depth + 1);
}

void BVH::inplace_partition(int nodeIdx, double splitPos, int axis, int & i){
  BVHNode & node = bvhNodes_.at(nodeIdx);
  int j = i + node.triCount-1;
  while (i <= j){
    if (triangles_.at(i)->centroid[axis] < splitPos){
      i++;
    }else{
      std::swap(triangles_.at(i), triangles_.at(j));
      j--;
    }
  }
}


// =============================================================================
// Private Methods - SoA Version
// =============================================================================

void BVH::updateNodeBoundsSoA(int nodeIdx){
  BVHNode& node = bvhNodes_.at(nodeIdx);
  node.bb_min_ = vec3(DBL_MAX);
  node.bb_max_ = vec3(-DBL_MAX);

  // Expand bounding box to contain all triangle vertices
  for (int i = node.firstTriIdx; i < node.firstTriIdx + node.triCount; i++){
    int i0 = data_->vertexIdx_[i*3];
    int i1 = data_->vertexIdx_[i*3+1];
    int i2 = data_->vertexIdx_[i*3+2];
    vec3 v0 = data_->vertexPos_[i0];
    vec3 v1 = data_->vertexPos_[i1];
    vec3 v2 = data_->vertexPos_[i2];
    node.bb_min_ = fmin(node.bb_min_, v0);
    node.bb_min_ = fmin(node.bb_min_, v1);
    node.bb_min_ = fmin(node.bb_min_, v2);
    node.bb_max_ = fmax(node.bb_max_, v0);
    node.bb_max_ = fmax(node.bb_max_, v1);
    node.bb_max_ = fmax(node.bb_max_, v2);
  }
}

void BVH::subdivideSoA(int nodeIdx, int depth){
  BVHNode& node = bvhNodes_.at(nodeIdx);

  // Debug output
  std::cout << "subdivide() depth: " << depth << ", "
            << "nodeIdx: " << nodeIdx << ", "
            << "node.triCount: " << node.triCount << std::endl;

  // Terminate recursion if node has few triangles
  if (node.triCount <= 2) return;

  // Determine split axis (cycle through x, y, z based on depth)
  int axis = depth % 3;
  double splitPos = medianSoA(axis, nodeIdx);

  // Partition triangles around split position
  int i = node.firstTriIdx;
  inplace_partitionSoA(nodeIdx, splitPos, axis, i);

  // Abort split if one side is empty
  int leftCount = i - node.firstTriIdx;
  if (leftCount == 0 || leftCount == node.triCount) return;

  // Create child nodes
  int leftChildIdx = nodesUsed_;
  int rightChildIdx = leftChildIdx + 1;
  nodesUsed_ = nodesUsed_ + 2;
  bvhNodes_[leftChildIdx].firstTriIdx = node.firstTriIdx;
  bvhNodes_[leftChildIdx].triCount = leftCount;
  bvhNodes_[rightChildIdx].firstTriIdx = i;
  bvhNodes_[rightChildIdx].triCount = node.triCount - leftCount;
  node.leftChildIdx = leftChildIdx;
  node.triCount = 0;
  updateNodeBoundsSoA(leftChildIdx);
  updateNodeBoundsSoA(rightChildIdx);

  // Recursively subdivide children
  subdivideSoA(leftChildIdx, depth + 1);
  subdivideSoA(rightChildIdx, depth + 1);
}

void BVH::inplace_partitionSoA(int nodeIdx, double splitPos, int axis, int& i){
  BVHNode & node = bvhNodes_.at(nodeIdx);
  int j = i + node.triCount-1;

  // Partition using two-pointer approach
  while (i <= j){
    // Compute centroid for triangle i
    int vi0 = data_->vertexIdx_[i*3];
    int vi1 = data_->vertexIdx_[i*3+1];
    int vi2 = data_->vertexIdx_[i*3+2];
    vec3 vp0 = data_->vertexPos_[vi0];
    vec3 vp1 = data_->vertexPos_[vi1];
    vec3 vp2 = data_->vertexPos_[vi2];
    vec3 centroid = (vp0 + vp1 + vp2) / 3.0;

    if (centroid[axis] < splitPos){
      i++;
    } else {
      // Swap triangle data in SoA arrays
      std::swap(data_->normals_[i], data_->normals_[j]);
      std::swap(data_->vertexIdx_[i*3], data_->vertexIdx_[j*3]);
      std::swap(data_->vertexIdx_[i*3+1], data_->vertexIdx_[j*3+1]);
      std::swap(data_->vertexIdx_[i*3+2], data_->vertexIdx_[j*3+2]);
      std::swap(data_->textureIdx_[i*3], data_->textureIdx_[j*3]);
      std::swap(data_->textureIdx_[i*3+1], data_->textureIdx_[j*3+1]);
      std::swap(data_->textureIdx_[i*3+2], data_->textureIdx_[j*3+2]);
      j--;
    }
  }
}

// =============================================================================
// Splitting Heuristics
// =============================================================================

double BVH::median(int axis, int nodeIdx) {
  BVHNode& node = bvhNodes_.at(nodeIdx);
  if (node.triCount <= 0) throw std::runtime_error("median: empty range");
  if (axis < 0 || axis > 2) throw std::out_of_range("axis must be 0..2");

  // Collect centroid coordinates along split axis
  std::vector<double> axis_pts(node.triCount);
  for (int i = 0; i < node.triCount; i++){
    axis_pts[i] = triangles_[node.firstTriIdx + i]->centroid[axis];
  }
  return median_inplace(axis_pts);
}

double BVH::medianSoA(int axis, int nodeIdx) {
  BVHNode& node = bvhNodes_.at(nodeIdx);
  if (node.triCount <= 0) throw std::runtime_error("median: empty range");
  if (axis < 0 || axis > 2) throw std::out_of_range("axis must be 0..2");

  // Collect centroid coordinates along split axis
  std::vector<double> axis_pts(node.triCount);
  for (int i = 0; i < node.triCount; i++){
    int idx = (node.firstTriIdx + i) * 3;
    int i0 = data_->vertexIdx_[idx];
    int i1 = data_->vertexIdx_[idx+1];
    int i2 = data_->vertexIdx_[idx+2];
    vec3 v0 = data_->vertexPos_[i0];
    vec3 v1 = data_->vertexPos_[i1];
    vec3 v2 = data_->vertexPos_[i2];
    vec3 centroid = (v0 + v1 + v2) / 3.0;
    axis_pts[i] = centroid[axis];
  }
  return median_inplace(axis_pts);
}

double BVH::median_inplace(std::vector<double>& a) {
    const size_t n = a.size();
    const size_t mid = n / 2;

    if (n % 2 == 1) {
        // Odd number of elements: return middle element
        std::nth_element(a.begin(), a.begin() + mid, a.end());
        return a[mid];
    } else {
        // Even number of elements: return average of two middle elements
        std::nth_element(a.begin(), a.begin() + mid, a.end());
        const double hi = a[mid];
        std::nth_element(a.begin(), a.begin() + (mid - 1), a.begin() + mid);
        const double lo = a[mid - 1];
        return 0.5 * (lo + hi);
    }
}
