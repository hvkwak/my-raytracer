// ============================================================================
// Computer Graphics(Graphische Datenverarbeitung) - TU Dortmund
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
#ifdef CUDA_ENABLED
#include <cuda_runtime.h>
#include "common/common.h"
#endif // CUDA_ENABLED
// =============================================================================
// Public Methods
// =============================================================================

BVH::~BVH(){
#ifdef CUDA_ENABLED
  if (d_bvhNodesSoA_){
    auto F = [&](auto *&p) {if (p) { cudaFree(p);p = nullptr;}};
    F(d_bvhNodesSoA_->bb_min_);
    F(d_bvhNodesSoA_->bb_max_);
    F(d_bvhNodesSoA_->triCount_);
    F(d_bvhNodesSoA_->leftChildIdx_);
    F(d_bvhNodesSoA_->firstTriIdx_);
    F(d_bvhNodesSoA_);
  }
#endif
}

void BVH::init(std::vector<Mesh*> &meshes) {

  std::cout << "Build BVH...";
  if (meshes.size() > 0){
    // Extract triangle data from meshes
    getData(meshes);

    // Allocate node pool
    bvhNodes_.resize(2 * triCount - 1);

    // Initialize root node
    BVHNode &root = bvhNodes_[rootNodeIdx_];
    root.leftChildIdx_ = 0;
    root.firstTriIdx_ = 0;
    root.triCount_ = triCount;
    updateNodeBounds(rootNodeIdx_);

    // Build BVH recursively
    subdivide(rootNodeIdx_, 1);

    isInitialized_ = true;

    // Debug: print first 50 nodes
    // for (int i = 0; i < 50 && i < bvhNodes_.size(); i++) {
    //   BVHNode node = bvhNodes_.at(i);
    //   std::cout << i << "-th Node"
    //             << " bb_min_: " << node.bb_min_
    //             << " bb_max_: " << node.bb_max_
    //             << " leftChildIdx: " << node.leftChildIdx_
    //             << " firstTriIdx: " << node.firstTriIdx_
    //             << " triCount: " << node.triCount_ << std::endl;
    // }
    std::cout << " done. \n" << std::flush;
  } else {
    std::cout << "No mesh available. BVH build stopped." << std::endl;
  }
}

bool BVH::isInitialized() const{
  return isInitialized_;
}

bool BVH::intersectAABB(const Ray & ray, const vec4 & bb_min_, const vec4 & bb_max_, double & tmin_) const{
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

  // tmin = std::max(tmin, tzmin);
  tmax = std::min(tmax, tzmax);

  // for left, right child check
  tmin_ = tmin;

  return tmax > 1e-5;  // Cull if farther than current closest hit
}


bool BVH::intersectBVH(const Ray& ray,
                       Material& intersection_material,
                       vec4 &intersection_point,
                       vec4 &intersection_normal,
                       double &intersection_distance,
                       int nodeIdx) const{
  const BVHNode & node = bvhNodes_[nodeIdx];

  // Early exit if ray doesn't intersect node's bounding box
  double dummy;
  if (!intersectAABB(ray, node.bb_min_, node.bb_max_, dummy)){
    return false;
  }

  if (node.triCount_ > 0) {
    // Leaf node - test all triangles
    double t;
    vec4 p, n, d;
    for (int i = node.firstTriIdx_; i < node.firstTriIdx_ + node.triCount_; i++){
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
    // Internal node - test children only if AABB hits
    bool hitLeft = false, hitRight = false;
    double dummy;

    if (intersectAABB(ray,
                      bvhNodes_[node.leftChildIdx_].bb_min_,
                      bvhNodes_[node.leftChildIdx_].bb_max_,
                      dummy)) {
      hitLeft = intersectBVH(ray,
                             intersection_material,
                             intersection_point,
                             intersection_normal,
                             intersection_distance,
                             node.leftChildIdx_);
    }

    if (intersectAABB(ray,
                      bvhNodes_[node.leftChildIdx_ + 1].bb_min_,
                      bvhNodes_[node.leftChildIdx_ + 1].bb_max_,
                      dummy)) {
      hitRight = intersectBVH(ray,
                              intersection_material,
                              intersection_point,
                              intersection_normal,
                              intersection_distance,
                              node.leftChildIdx_ + 1);
    }

    return hitLeft || hitRight;
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
      vec4 vertex0 = mesh->vertices_.at(tri.i0).position;
      vec4 vertex1 = mesh->vertices_.at(tri.i1).position;
      vec4 vertex2 = mesh->vertices_.at(tri.i2).position;
      vec4 centroid = (vertex0 + vertex1 + vertex2) / 3.0;
      tri.centroid = centroid;
      tri.meshIdx = i;
      triangles_.push_back(&tri);
    }
    triCount = triCount + mesh->triangles_.size();
    i++;
  }
}

void BVH::updateNodeBounds(int nodeIdx){
  BVHNode& node = bvhNodes_[nodeIdx];
  node.bb_min_ = vec4(DBL_MAX);
  node.bb_max_ = vec4(-DBL_MAX);

  // Expand bounding box to contain all triangle vertices
  for (int i = node.firstTriIdx_; i < node.firstTriIdx_ + node.triCount_; i++){
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
  BVHNode& node = bvhNodes_[nodeIdx];

  // Terminate recursion if node has few triangles
  if (node.triCount_ <= 2) return;

  // Determine split axis (cycle through x, y, z based on depth)
  int axis = depth % 3;
  double splitPos = median(axis, nodeIdx);

  // Partition triangles around split position
  int i = node.firstTriIdx_;
  inplace_partition(nodeIdx, splitPos, axis, i);

  // Abort split if one side is empty
  int leftCount = i - node.firstTriIdx_;
  if (leftCount == 0 || leftCount == node.triCount_) return;

  // Create child nodes
  int leftChildIdx = nodesUsed_;
  int rightChildIdx = leftChildIdx + 1;
  nodesUsed_ = nodesUsed_ + 2;
  bvhNodes_[leftChildIdx].firstTriIdx_ = node.firstTriIdx_;
  bvhNodes_[leftChildIdx].triCount_ = leftCount;
  bvhNodes_[rightChildIdx].firstTriIdx_ = i;
  bvhNodes_[rightChildIdx].triCount_ = node.triCount_ - leftCount;
  node.leftChildIdx_ = leftChildIdx;
  node.triCount_ = 0;
  updateNodeBounds(leftChildIdx);
  updateNodeBounds(rightChildIdx);

  // Recursively subdivide children
  subdivide(leftChildIdx, depth + 1);
  subdivide(rightChildIdx, depth + 1);
}

void BVH::inplace_partition(int nodeIdx, double splitPos, int axis, int & i){
  BVHNode & node = bvhNodes_[nodeIdx];
  int j = i + node.triCount_-1;
  while (i <= j){
    if (triangles_.at(i)->centroid[axis] < splitPos){
      i++;
    }else{
      std::swap(triangles_.at(i), triangles_.at(j));
      j--;
    }
  }
}

double BVH::median(int axis, int nodeIdx) {
  BVHNode& node = bvhNodes_[nodeIdx];
  if (node.triCount_ <= 0) throw std::runtime_error("median: empty range");
  if (axis < 0 || axis > 2) throw std::out_of_range("axis must be 0..2");

  // Collect centroid coordinates along split axis
  std::vector<double> axis_pts(node.triCount_);
  for (int i = 0; i < node.triCount_; i++){
    axis_pts[i] = triangles_[node.firstTriIdx_ + i]->centroid[axis];
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


// =============================================================================
// CUDA - SoA Version
// =============================================================================

#ifdef CUDA_ENABLED
void BVH::initSoA(std::vector<Mesh*> &meshes, Data* data)
{
  std::cout << "Build BVH...";

  if (meshes.size() > 0){

    data_ = data;
    triCount = data_->tVertexIdxCount_/3;

    // Allocate node pool in SoA, initialize with invalid values
    CHECK(cudaMallocManaged(&d_bvhNodesSoA_, sizeof(BVHNodes_SoA)));
    CHECK(cudaMallocManaged(&d_bvhNodesSoA_->bb_min_, (2*triCount-1)* sizeof(vec4)));
    CHECK(cudaMallocManaged(&d_bvhNodesSoA_->bb_max_, (2*triCount-1)* sizeof(vec4)));
    CHECK(cudaMallocManaged(&d_bvhNodesSoA_->triCount_, (2*triCount-1)* sizeof(int)));
    CHECK(cudaMallocManaged(&d_bvhNodesSoA_->leftChildIdx_, (2*triCount-1)* sizeof(int)));
    CHECK(cudaMallocManaged(&d_bvhNodesSoA_->firstTriIdx_, (2*triCount-1)* sizeof(int)));

    // d_bvhNodesSoA_->bb_min_.resize(2*triCount-1, vec4(-DBL_MAX));
    // d_bvhNodesSoA_->bb_max_.resize(2*triCount-1, vec4(DBL_MAX));
    // d_bvhNodesSoA_->triCount_.resize(2*triCount-1, -1);
    // d_bvhNodesSoA_->leftChildIdx_.resize(2*triCount-1, -1);
    // d_bvhNodesSoA_->firstTriIdx_.resize(2*triCount-1, -1);

    // bvhNodes_.resize(2*triCount-1);

    // Initialize root node
    d_bvhNodesSoA_->leftChildIdx_[rootNodeIdx_] = 0;
    d_bvhNodesSoA_->firstTriIdx_[rootNodeIdx_] = 0;
    d_bvhNodesSoA_->triCount_[rootNodeIdx_] = triCount;
    // BVHNode & root = bvhNodes_[rootNodeIdx_];
    // root.leftChildIdx_ = 0;
    // root.firstTriIdx_ = 0;
    // root.triCount_ = triCount;
    updateNodeBoundsSoA(rootNodeIdx_);

    // Build BVH recursively
    subdivideSoA(rootNodeIdx_, 1);

    isInitialized_ = true;
    // Debug: print first 50 nodes
    // for (int i = 0; i < 50; i++) {
    //   std::cout << i << "-th Node"
    //             << " bb_min_: " << d_bvhNodesSoA_->bb_min_.at(i)
    //             << " bb_max_: " << d_bvhNodesSoA_->bb_max_.at(i)
    //             << " leftChildIdx: " << d_bvhNodesSoA_->leftChildIdx_.at(i)
    //             << " firstTriIdx: " << d_bvhNodesSoA_->firstTriIdx_.at(i)
    //             << " triCount: " << d_bvhNodesSoA_->triCount_.at(i) << std::endl;
    // }
    std::cout << " done. \n" << std::flush;
  } else {
    std::cout << "No mesh available. BVH build stopped." << std::endl;
  }
}

void BVH::updateNodeBoundsSoA(int nodeIdx){
  d_bvhNodesSoA_->bb_min_[nodeIdx] = vec4(DBL_MAX);
  d_bvhNodesSoA_->bb_max_[nodeIdx] = vec4(-DBL_MAX);

  // Expand bounding box to contain all triangle vertices
  for (int i = d_bvhNodesSoA_->firstTriIdx_[nodeIdx]; i <d_bvhNodesSoA_->firstTriIdx_[nodeIdx]+d_bvhNodesSoA_->triCount_[nodeIdx]; i++){
    int i0 = data_->vertexIdx_[i*3];
    int i1 = data_->vertexIdx_[i*3+1];
    int i2 = data_->vertexIdx_[i*3+2];
    vec4 v0 = data_->vertexPos_[i0];
    vec4 v1 = data_->vertexPos_[i1];
    vec4 v2 = data_->vertexPos_[i2];
    d_bvhNodesSoA_->bb_min_[nodeIdx] = fmin(d_bvhNodesSoA_->bb_min_[nodeIdx], v0);
    d_bvhNodesSoA_->bb_min_[nodeIdx] = fmin(d_bvhNodesSoA_->bb_min_[nodeIdx], v1);
    d_bvhNodesSoA_->bb_min_[nodeIdx] = fmin(d_bvhNodesSoA_->bb_min_[nodeIdx], v2);
    d_bvhNodesSoA_->bb_max_[nodeIdx] = fmax(d_bvhNodesSoA_->bb_max_[nodeIdx], v0);
    d_bvhNodesSoA_->bb_max_[nodeIdx] = fmax(d_bvhNodesSoA_->bb_max_[nodeIdx], v1);
    d_bvhNodesSoA_->bb_max_[nodeIdx] = fmax(d_bvhNodesSoA_->bb_max_[nodeIdx], v2);
  }
}
void BVH::subdivideSoA(int nodeIdx, int depth){
  // Debug output
  // std::cout << "subdivide() depth: " << depth << ", "
  //           << "nodeIdx: " << nodeIdx << ", "
  //           << "node.triCount: " << d_bvhNodesSoA_->triCount_.at(nodeIdx) << std::endl;

  // Terminate recursion if node has few triangles
  if (d_bvhNodesSoA_->triCount_[nodeIdx] <= 2) return;

  // Determine split axis (cycle through x, y, z based on depth)
  int axis = depth % 3;
  double splitPos = medianSoA(axis, nodeIdx);

  // Partition triangles around split position
  int i = d_bvhNodesSoA_->firstTriIdx_[nodeIdx];
  inplace_partitionSoA(nodeIdx, splitPos, axis, i);

  // Abort split if one side is empty
  int leftCount = i - d_bvhNodesSoA_->firstTriIdx_[nodeIdx];
  if (leftCount == 0 || leftCount == d_bvhNodesSoA_->triCount_[nodeIdx]) return;

  // Create child nodes
  int leftChildIdx = nodesUsed_;
  int rightChildIdx = leftChildIdx + 1;
  nodesUsed_ = nodesUsed_ + 2;
  d_bvhNodesSoA_->firstTriIdx_[leftChildIdx] = d_bvhNodesSoA_->firstTriIdx_[nodeIdx];
  d_bvhNodesSoA_->triCount_[leftChildIdx] = leftCount;
  d_bvhNodesSoA_->firstTriIdx_[rightChildIdx] = i;
  d_bvhNodesSoA_->triCount_[rightChildIdx] = d_bvhNodesSoA_->triCount_[nodeIdx] - leftCount;
  d_bvhNodesSoA_->leftChildIdx_[nodeIdx] = leftChildIdx;
  d_bvhNodesSoA_->triCount_[nodeIdx] = 0;

  updateNodeBoundsSoA(leftChildIdx);
  updateNodeBoundsSoA(rightChildIdx);

  // Recursively subdivide children
  subdivideSoA(leftChildIdx, depth + 1);
  subdivideSoA(rightChildIdx, depth + 1);
}

void BVH::inplace_partitionSoA(int nodeIdx, double splitPos, int axis, int& i){
  int j = i + d_bvhNodesSoA_->triCount_[nodeIdx]-1;
  // Partition using two-pointer approach
  while (i <= j){
    // Compute centroid for triangle i
    int vi0 = data_->vertexIdx_[i*3];
    int vi1 = data_->vertexIdx_[i*3+1];
    int vi2 = data_->vertexIdx_[i*3+2];
    vec4 vp0 = data_->vertexPos_[vi0];
    vec4 vp1 = data_->vertexPos_[vi1];
    vec4 vp2 = data_->vertexPos_[vi2];
    vec4 centroid = (vp0 + vp1 + vp2) / 3.0;
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
      vi0 = data_->vertexIdx_[j * 3];
      vi1 = data_->vertexIdx_[j * 3 + 1];
      vi2 = data_->vertexIdx_[j * 3 + 2];
      vp0 = data_->vertexPos_[vi0];
      vp1 = data_->vertexPos_[vi1];
      vp2 = data_->vertexPos_[vi2];
      j--;
    }
  }
}



double BVH::medianSoA(int axis, int nodeIdx) {
  if (d_bvhNodesSoA_->triCount_[nodeIdx] <= 0) throw std::runtime_error("median: empty range");
  if (axis < 0 || axis > 2) throw std::out_of_range("axis must be 0..2");

  // Collect centroid coordinates along split axis
  std::vector<double> axis_pts(d_bvhNodesSoA_->triCount_[nodeIdx]);
  for (int i = 0; i < d_bvhNodesSoA_->triCount_[nodeIdx]; i++){
    int idx = (d_bvhNodesSoA_->firstTriIdx_[nodeIdx] + i) * 3;
    int i0 = data_->vertexIdx_[idx];
    int i1 = data_->vertexIdx_[idx+1];
    int i2 = data_->vertexIdx_[idx+2];
    vec4 v0 = data_->vertexPos_[i0];
    vec4 v1 = data_->vertexPos_[i1];
    vec4 v2 = data_->vertexPos_[i2];
    vec4 centroid = (v0 + v1 + v2) / 3.0;
    axis_pts[i] = centroid[axis];
  }
  return median_inplace(axis_pts);
}
#endif // CUDA_ENABLED
