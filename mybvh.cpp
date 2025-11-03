#include "mybvh.h"
#include <algorithm>
#include <cfloat>
#include <math.h>
#include <vector>

/*--- Public -----------------------------------------------------------------*/
BVH::~BVH(){}

void BVH::init(std::vector<Mesh*> &meshes) {

  if (meshes.size() > 0){
    // filter mesh objects
    getData(meshes);

    bvhNodes_.resize(2 * triCount - 1);

    // build BVH
    BVHNode &root = bvhNodes_[rootNodeIdx_];
    root.leftChildIdx = 0;
    root.firstTriIdx = 0;
    root.triCount = triCount;
    updateNodeBounds(rootNodeIdx_);

    // subdivide recursively
    subdivide(rootNodeIdx_, 1);

    // initialized
    isInitialized_ = true;

    // print out the BVH
    // for (int i = 0; i < 50 /*bvhNodes_.size()*/; i++) {
    //   BVHNode node = bvhNodes_.at(i);
    //   std::cout << i << "-th Node"
    //             << " bb_min_: " << node.bb_min_
    //             << " bb_max_: " << node.bb_max_
    //             << " leftChildIdx: " << node.leftChildIdx
    //             << " firstTriIdx: " << node.firstTriIdx
    //             << " triCount: " << node.triCount << std::endl;
    // }
  }else{
    std::cout << "no mesh available. BVH build stop." << std::endl;
  }
}

void BVH::initSoA(std::vector<Mesh*> &meshes, Data* data)
{
  if (meshes.size() > 0){
    meshes_ = meshes;
    data_ = data;
    triCount = data_->vertexIdxCount_.size()/3;
    bvhNodes_.resize(2 * triCount - 1);

    // build BVH
    BVHNode &root = bvhNodes_[rootNodeIdx_];
    root.leftChildIdx = 0;
    root.firstTriIdx = 0;
    root.triCount = triCount;
    updateNodeBoundsSoA(rootNodeIdx_);

    // subdivide recursively
    subdivideSoA(rootNodeIdx_, 1);

    // initialized
    isInitialized_ = true;

  }else{
    std::cout << "no mesh available. BVH build stop." << std::endl;
  }
}

bool BVH::isInitialized() const{
  return isInitialized_;
}

bool BVH::intersectAABB(const Ray & ray, const vec3 & bb_min_, const vec3 & bb_max_) const{

  /** copy from mymesh.cpp :(, but BVH nodes without Mesh need this. */
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

  return tmax > 1e-5; // Check if intersection is in front of ray
}


/**
 * @brief intersect ray with the BVH
 *
 */
bool BVH::intersectBVH(const Ray& ray,
                       Material& intersection_material,
                       vec3 &intersection_point,
                       vec3 &intersection_normal,
                       double &intersection_distance,
                       int nodeIdx) const{
  const BVHNode & node = bvhNodes_.at(nodeIdx);
  if (!intersectAABB(ray, node.bb_min_, node.bb_max_)){
    return false;
  }

  if (node.triCount > 0) {
    // Leaf
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
  if (!intersectAABB(ray, node.bb_min_, node.bb_max_)){
    return false;
  }

  if (node.triCount > 0) {
    // Leaf
    double t;
    vec3 p, n, d;
    for (int i = node.firstTriIdx; i < node.firstTriIdx + node.triCount; i++){
      int vi = data_->vertexIdx_.at(i*3); // vertex index from Tri.Index
      Mesh* mesh = data_->meshes_.at(vi);
      // TODO: build a triangle
      Triangle tri{.i0 = a,
                   .i1 = b,
                   .i2 = c,
                   .normal = n,
                   .iuv0 = u0,
                   .iuv1 = u1,
                   .iuv2 = u2,
                   .centroid = c3,
                   .meshIdx = m};
      if (mesh->intersect_triangle(tri, ray, p, n, d, t)) {
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
    bool isLeft = intersectBVH(ray,intersection_material,intersection_point,intersection_normal, intersection_distance, node.leftChildIdx);
    bool isRight = intersectBVH(ray,intersection_material,intersection_point,intersection_normal, intersection_distance, node.leftChildIdx + 1);
    return isLeft || isRight;
  }
}
/*--- Private::no SoA --------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 * @return
 */
void BVH::getData(std::vector<Mesh*> &meshes){
  meshes_ = meshes;
  int i = 0;
  for (Mesh* mesh : meshes_){
    for (Triangle &tri : mesh->triangles_) {
      // for each triangle
      vec3 vertex0 = mesh->vertices_.at(tri.i0).position;
      vec3 vertex1 = mesh->vertices_.at(tri.i1).position;
      vec3 vertex2 = mesh->vertices_.at(tri.i2).position;
      vec3 centroid = (vertex0 + vertex1 + vertex2)/3.0;
      // std::cout << "centroid: " << centroid << std::endl;
      tri.centroid = centroid;
      tri.meshIdx = i;
      triangles_.push_back(&tri); // Push address of each triangle
    }
    triCount = triCount + mesh->triangles_.size();
    i++;
  }
}


/**
 * @brief
 *
 * @param
 * @return
 */
void BVH::updateNodeBounds(int nodeIdx){

  BVHNode& node = bvhNodes_.at(nodeIdx);
  node.bb_min_ = vec3(DBL_MAX);
  node.bb_max_ = vec3(-DBL_MAX);
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
  //std::cout << "subdivide() depth: " << depth << std::endl;
  // Terminate recursion
  BVHNode& node = bvhNodes_.at(nodeIdx);
  if (node.triCount <= 2) return;

  // split axis and position
  int axis = depth % 3;
  double splitPos = median(axis, nodeIdx);
  // vec3 length = node.bb_max_ - node.bb_min_;
  // if (length[1] > length[0]) axis = 1;
  // if (length[2] > length[axis]) axis = 2;
  // float splitPos = node.bb_min_[axis] + length[axis]*0.5f;

  // in-place partition
  int i = node.firstTriIdx;
  inplace_partition(nodeIdx, splitPos, axis, i);
  // int j = i + node.triCount-1;
  // while (i <= j){
  //   if (triangles.at(i)->centroid[axis] < splitPos){
  //     i++;
  //   }else{
  //     std::swap(triangles.at(i), triangles.at(j));
  //     j--;
  //   }
  // }

  // abort split if one of the side is empty
  int leftCount = i - node.firstTriIdx;
  if (leftCount == 0 || leftCount == node.triCount) return;

  // create child nodes
  int leftChildIdx = nodesUsed_;
  int rightChildIdx = leftChildIdx + 1;
  nodesUsed_ = nodesUsed_ + 2;
  bvhNodes_[leftChildIdx].firstTriIdx = node.firstTriIdx;
  bvhNodes_[leftChildIdx].triCount = leftCount;
  bvhNodes_[rightChildIdx].firstTriIdx = i;
  bvhNodes_[rightChildIdx].triCount = node.triCount-leftCount;
  node.leftChildIdx = leftChildIdx;
  node.triCount = 0;
  updateNodeBounds(leftChildIdx);
  updateNodeBounds(rightChildIdx);

  // recurse
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


/*--- Private::methods for SoA --------------------------------------------------------*/

void BVH::updateNodeBoundsSoA(int nodeIdx){
  BVHNode& node = bvhNodes_.at(nodeIdx);
  node.bb_min_ = vec3(DBL_MAX);
  node.bb_max_ = vec3(-DBL_MAX);
  for (int i = node.firstTriIdx; i < node.firstTriIdx + node.triCount; i++){
    int vi = data_->vertexIdx_.at(i*3); // vertex index from Tri.Index
    vec3 v0 = data_->vertexPos_.at(vi+0);
    vec3 v1 = data_->vertexPos_.at(vi+1);
    vec3 v2 = data_->vertexPos_.at(vi+2);
    node.bb_min_ = fmin(node.bb_min_, v0);
    node.bb_min_ = fmin(node.bb_min_, v1);
    node.bb_min_ = fmin(node.bb_min_, v2);
    node.bb_max_ = fmax(node.bb_max_, v0);
    node.bb_max_ = fmax(node.bb_max_, v1);
    node.bb_max_ = fmax(node.bb_max_, v2);
 }
}

void BVH::subdivideSoA(int nodeIdx, int depth){
  //std::cout << "subdivide() depth: " << depth << std::endl;
  // Terminate recursion
  BVHNode& node = bvhNodes_.at(nodeIdx);
  if (node.triCount <= 2) return;

  // split axis and position
  int axis = depth % 3;
  double splitPos = medianSoA(axis, nodeIdx);

  // in-place partition
  int i = node.firstTriIdx;
  inplace_partitionSoA(nodeIdx, splitPos, axis, i);

  // abort split if one of the side is empty
  int leftCount = i - node.firstTriIdx;
  if (leftCount == 0 || leftCount == node.triCount) return;

  // create child nodes
  int leftChildIdx = nodesUsed_;
  int rightChildIdx = leftChildIdx + 1;
  nodesUsed_ = nodesUsed_ + 2;
  bvhNodes_[leftChildIdx].firstTriIdx = node.firstTriIdx;
  bvhNodes_[leftChildIdx].triCount = leftCount;
  bvhNodes_[rightChildIdx].firstTriIdx = i;
  bvhNodes_[rightChildIdx].triCount = node.triCount-leftCount;
  node.leftChildIdx = leftChildIdx;
  node.triCount = 0;
  updateNodeBoundsSoA(leftChildIdx);
  updateNodeBoundsSoA(rightChildIdx);

  // recurse
  subdivideSoA(leftChildIdx, depth + 1);
  subdivideSoA(rightChildIdx, depth + 1);
}

void BVH::inplace_partitionSoA(int nodeIdx, double splitPos, int axis, int& i){
  BVHNode & node = bvhNodes_.at(nodeIdx);
  int j = i + node.triCount-1;
  while (i <= j){
    int idx = (i + node.firstTriIdx) * 3;
    int jdx = (j + node.firstTriIdx) * 3;
    int vi = data_->vertexIdx_.at(idx); // vertex index from Tri.Index
    vec3 v0 = data_->vertexPos_.at(vi + 0);
    vec3 v1 = data_->vertexPos_.at(vi + 1);
    vec3 v2 = data_->vertexPos_.at(vi + 2);
    vec3 centroid = (v0 + v1 + v2) / 3.0;
    if (centroid[axis] < splitPos){
      i++;
    }else{
      std::swap(data_->vertexIdx_.at(idx), data_->vertexIdx_.at(jdx));
      std::swap(data_->vertexIdx_.at(idx+1), data_->vertexIdx_.at(jdx+1));
      std::swap(data_->vertexIdx_.at(idx+2), data_->vertexIdx_.at(jdx+2));
      std::swap(data_->textureIdx_.at(idx), data_->textureIdx_.at(jdx));
      std::swap(data_->textureIdx_.at(idx+1), data_->textureIdx_.at(jdx+1));
      std::swap(data_->textureIdx_.at(idx+2), data_->textureIdx_.at(jdx+2));
      j--;
    }
  }
}


/*---------------------------------------------------------*/

/**
 * @brief returns median of the triangles.centroid
 *
 * @param axis: axis of the centroid
 * @return
 */
double BVH::median(int axis, int nodeIdx) {
  BVHNode& node = bvhNodes_.at(nodeIdx);
  if (node.triCount <= 0) throw std::runtime_error("median: empty range");
  if (axis < 0 || axis > 2) throw std::out_of_range("axis must be 0..2");
  std::vector<double> axis_pts(node.triCount);
  for (int i = 0; i < node.triCount; i++){
    axis_pts[i] = triangles_.at(node.firstTriIdx + i)->centroid[axis];
  }
  return median_inplace(axis_pts);
}

/**
 * @brief returns median of the triangles.centroid
 *
 * @param axis: axis of the centroid
 * @return
 */
double BVH::medianSoA(int axis, int nodeIdx) {
  BVHNode& node = bvhNodes_.at(nodeIdx);
  if (node.triCount <= 0) throw std::runtime_error("median: empty range");
  if (axis < 0 || axis > 2) throw std::out_of_range("axis must be 0..2");
  std::vector<double> axis_pts(node.triCount);
  for (int i = 0; i < node.triCount; i++){
    int idx = (node.firstTriIdx+i)*3;
    int vi = data_->vertexIdx_.at(idx); // vertex index from Tri.Index
    vec3 v0 = data_->vertexPos_.at(vi+0);
    vec3 v1 = data_->vertexPos_.at(vi+1);
    vec3 v2 = data_->vertexPos_.at(vi+2);
    vec3 centroid = (v0 + v1 + v2)/3.0;
    axis_pts[i] = centroid[axis];
  }
  return median_inplace(axis_pts);
}


/**
 * @brief
 *
 * @param
 * @return
 */
double BVH::median_inplace(std::vector<double>& a) {
    // // Optional: drop NaNs if they might appear (NaNs break strict-weak-ordering)
    // a.erase(std::remove_if(a.begin(), a.end(),
    //                        [](double x){ return std::isnan(x); }),
    //         a.end());
    // if (a.empty()) throw std::runtime_error("median of empty vector");

    const size_t n   = a.size();
    const size_t mid = n / 2;

    if (n % 2 == 1) {                  // odd
        std::nth_element(a.begin(), a.begin() + mid, a.end());
        return a[mid];
    } else {                           // even: average of two middle values
        std::nth_element(a.begin(), a.begin() + mid, a.end());
        const double hi = a[mid];
        // get the largest element of the lower half
        std::nth_element(a.begin(), a.begin() + (mid - 1), a.begin() + mid);
        const double lo = a[mid - 1];
        return 0.5 * (lo + hi);
    }
}
