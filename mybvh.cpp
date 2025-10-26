#include "mybvh.h"
#include <algorithm>
#include <cfloat>
#include <math.h>
#include <vector>

BVH::~BVH(){}

void BVH::init(std::vector<Mesh*> &meshData) {

  // filter mesh objects
  getData(meshData);
  int triCount = triangles.size();
  if (triCount > 2){
    bvhNodes.resize(2 * triCount - 1);

    // build BVH
    BVHNode &root = bvhNodes[rootNodeIdx];
    root.leftChildIdx = 0;
    root.firstTriIdx = 0;
    root.triCount = triCount;
    updateNodeBounds(rootNodeIdx);

    // subdivide recursively
    subdivide(rootNodeIdx, 1);

    // initialized
    isInitialized_ = true;

    // print out the BVH
    // for (int i = 0; i < 50 /*bvhNodes.size()*/; i++) {
    //   BVHNode node = bvhNodes.at(i);
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

bool BVH::isInitialized() const{
  return isInitialized_;
}

void BVH::subdivide(int nodeIdx, int depth){
  //std::cout << "subdivide() depth: " << depth << std::endl;
  // Terminate recursion
  BVHNode& node = bvhNodes.at(nodeIdx);
  if (node.triCount <= 2) return;

  // split axis and position
  int axis = depth % 3;
  double splitPos = median(axis, node.firstTriIdx, node.triCount);
  // vec3 length = node.bb_max_ - node.bb_min_;
  // if (length[1] > length[0]) axis = 1;
  // if (length[2] > length[axis]) axis = 2;
  // float splitPos = node.bb_min_[axis] + length[axis]*0.5f;

  // in-place partition
  int i = node.firstTriIdx;
  int j = i + node.triCount-1;
  while (i <= j){
    if (triangles.at(i)->centroid[axis] < splitPos){
      i++;
    }else{
      std::swap(triangles.at(i), triangles.at(j));
      j--;
    }
  }

  // abort split if one of the side is empty
  int leftCount = i - node.firstTriIdx;
  if (leftCount == 0 || leftCount == node.triCount) return;

  // create child nodes
  int leftChildIdx = nodesUsed;
  int rightChildIdx = leftChildIdx + 1;
  nodesUsed = nodesUsed + 2;
  bvhNodes[leftChildIdx].firstTriIdx = node.firstTriIdx;
  bvhNodes[leftChildIdx].triCount = leftCount;
  bvhNodes[rightChildIdx].firstTriIdx = i;
  bvhNodes[rightChildIdx].triCount = node.triCount-leftCount;
  node.leftChildIdx = leftChildIdx;
  node.triCount = 0;
  updateNodeBounds(leftChildIdx);
  updateNodeBounds(rightChildIdx);

  // recurse
  subdivide(leftChildIdx, depth + 1);
  subdivide(rightChildIdx, depth + 1);
}

/**
 * @brief
 *
 * @param
 * @return
 */
void BVH::getData(std::vector<Mesh*> &meshData){
  meshes = meshData; // TODO: check if it is ok
  int i = 0;
  for (Mesh* mesh : meshes){
    for (Mesh::Triangle &tri : mesh->triangles_) {
      // for each triangle
      vec3 vertex0 = mesh->vertices_.at(tri.i0).position;
      vec3 vertex1 = mesh->vertices_.at(tri.i1).position;
      vec3 vertex2 = mesh->vertices_.at(tri.i2).position;
      vec3 centroid = (vertex0 + vertex1 + vertex2)/3.0;
      // std::cout << "centroid: " << centroid << std::endl;
      tri.centroid = centroid;
      tri.meshIdx = i;
      triangles.push_back(&tri); // Push address of each triangle
    }
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

  BVHNode& node = bvhNodes.at(nodeIdx);
  node.bb_min_ = vec3(DBL_MAX);
  node.bb_max_ = vec3(-DBL_MAX);
  for (int i = node.firstTriIdx; i < node.firstTriIdx + node.triCount; i++){
    Mesh::Triangle* tri = triangles.at(i);
    Mesh* mesh = meshes.at(tri->meshIdx);
    node.bb_min_ = fmin(node.bb_min_, mesh->vertices_.at(tri->i0).position);
    node.bb_min_ = fmin(node.bb_min_, mesh->vertices_.at(tri->i1).position);
    node.bb_min_ = fmin(node.bb_min_, mesh->vertices_.at(tri->i2).position);
    node.bb_max_ = fmax(node.bb_max_, mesh->vertices_.at(tri->i0).position);
    node.bb_max_ = fmax(node.bb_max_, mesh->vertices_.at(tri->i1).position);
    node.bb_max_ = fmax(node.bb_max_, mesh->vertices_.at(tri->i2).position);
 }
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
 * @brief
 *
 * @param t - probe parameter t
 *        intersection_distance - best t so far
 *        intersection
 * @return
 */
bool BVH::intersectBVH(const Ray& ray,
                       Material& intersection_material,
                       vec3 &intersection_point,
                       vec3 &intersection_normal,
                       double &intersection_distance,
                       int nodeIdx) const{
  const BVHNode & node = bvhNodes.at(nodeIdx);
  if (!intersectAABB(ray, node.bb_min_, node.bb_max_)){
    return false;
  }

  if (node.triCount > 0) {
    // Leaf
    double t;
    vec3 p, n, d;
    for (int i = node.firstTriIdx; i < node.firstTriIdx + node.triCount; i++){
      Mesh::Triangle* tri = triangles.at(i);
      Mesh* mesh = meshes.at(tri->meshIdx);
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

double BVH::median(int axis, int firstTriIdx, int triCount) {
  if (triCount <= 0) throw std::runtime_error("median: empty range");
  if (axis < 0 || axis > 2) throw std::out_of_range("axis must be 0..2");
  std::vector<double> axis_pts(triCount);
  for (int i = 0; i < triCount; i++){
    axis_pts[i] = triangles.at(firstTriIdx + i)->centroid[axis];
  }
  return median_inplace(axis_pts);
}
