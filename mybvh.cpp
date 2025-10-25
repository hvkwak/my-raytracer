#include "mybvh.h"
#include <cfloat>
#include <math.h>

BVH::~BVH(){}

void BVH::init(std::vector<Object_ptr> &objects) {

  // filter mesh objects
  filterMesh(objects);
  int meshCount = meshes.size();
  if (meshCount > 2){
    bvhNode.resize(2 * meshCount - 1);

    // build BVH
    BVHNode &root = bvhNode[rootNodeIdx];
    root.leftChildIdx = 0;
    root.firstMeshIdx = 0;
    root.meshCount = meshCount;
    updateNodeBounds(rootNodeIdx);

    // subdivide recursively
    subDivide(rootNodeIdx);

    // initialized
    isInitialized_ = true;

    // print out the BVH
    for (int i = 0; i < bvhNode.size(); i++) {
      BVHNode node = bvhNode.at(i);
      std::cout << i << "-th Node"
                << " bb_min_: " << node.bb_min_
                << " bb_max_: " << node.bb_max_
                << " leftChildIdx: " << node.leftChildIdx
                << " firstMeshIdx: " << node.firstMeshIdx
                << " meshCount: " << node.meshCount << std::endl;
    }
  }else{
    std::cout << "no mesh available. BVH build stop." << std::endl;
  }
}

bool BVH::isInitialized() const{
  return isInitialized_;
}

void BVH::subDivide(int nodeIdx){

  // Terminate recursion
  BVHNode& node = bvhNode.at(nodeIdx);
  if (node.meshCount <= 2) return;

  // split axis and position
  vec3 length = node.bb_max_ - node.bb_min_;
  int axis = 0;
  if (length[1] > length[0]) axis = 1;
  if (length[2] > length[axis]) axis = 2;
  float splitPos = node.bb_min_[axis] + length[axis]*0.5f;

  // in-place partition
  int i = node.firstMeshIdx;
  int j = i + node.meshCount-1;
  while (i <= j){
    Mesh* mesh = meshes.at(i);
    std::cout << meshes.at(i)->centroid[axis] << std::endl;
    if (meshes.at(i)->centroid[axis] < splitPos){
      i++;
    }else{
      std::swap(meshes.at(i), meshes.at(j));
      j--;
    }
  }

  // abort split if one of the side is empty
  int leftCount = i - node.firstMeshIdx;
  if (leftCount == 0 || leftCount == node.meshCount) return;

  // create child nodes
  int leftChildIdx = nodesUsed;
  int rightChildIdx = leftChildIdx + 1;
  nodesUsed = nodesUsed + 2;
  bvhNode[leftChildIdx].firstMeshIdx = node.firstMeshIdx;
  bvhNode[leftChildIdx].meshCount = leftCount;
  bvhNode[rightChildIdx].firstMeshIdx = i;
  bvhNode[rightChildIdx].meshCount = node.meshCount-leftCount;
  node.leftChildIdx = leftChildIdx;
  node.meshCount = 0;
  updateNodeBounds(leftChildIdx);
  updateNodeBounds(rightChildIdx);

  // recurse
  subDivide(leftChildIdx);
  subDivide(rightChildIdx);
}

/**
 * @brief
 *
 * @param
 * @return
 */
void BVH::filterMesh(std::vector<Object_ptr> &objects){
  for (Object_ptr object : objects){
    auto *mesh = dynamic_cast<Mesh*>(object);
    if (mesh != nullptr){
      meshes.push_back(mesh);
    }
  }
}

/**
 * @brief
 *
 * @param
 * @return
 */
void BVH::updateNodeBounds(int nodeIdx){

  BVHNode& node = bvhNode.at(nodeIdx);
  node.bb_min_ = vec3(DBL_MAX);
  node.bb_max_ = vec3(-DBL_MAX);
  for (int i = node.firstMeshIdx; i < node.firstMeshIdx + node.meshCount; i++){
    Mesh* mesh = meshes.at(i);
    node.bb_min_ = fmin(node.bb_min_, mesh->bb_min_);
    node.bb_max_ = fmax(node.bb_max_, mesh->bb_max_);
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
  const BVHNode & node = bvhNode.at(nodeIdx);
  if (!intersectAABB(ray, node.bb_min_, node.bb_max_)){
    return false;
  }

  if (node.meshCount > 0) {
    // Leaf
    double t;
    vec3 p, n, d;
    for (int i = node.firstMeshIdx; i < node.firstMeshIdx + node.meshCount; i++){
      Mesh* mesh = meshes.at(i);
      if (mesh->intersect(ray, p, n, d, t)) {
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
