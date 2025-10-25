#include "utils/vec3.h"
#include "utils/Ray.h"
#include "utils/Material.h"
#include "Mesh.h"
#include <math.h>

class BVH {

public:

  BVH() = default;
  ~BVH();
  void init(std::vector<Object_ptr> &objects);
  bool isInitialized() const;
  bool intersectAABB(const Ray & ray, const vec3 & bb_min_, const vec3 & bb_max_) const;
  bool intersectBVH(const Ray &ray,
                    Material& intersection_material,
                    vec3 &intersection_point,
                    vec3 &intersection_normal,
                    double &intersection_distance,
                    int nodeIdx) const;


private:
  // define BVH Node
  struct BVHNode {
    /// point of the bounding box
    vec3 bb_min_, bb_max_;
    int leftChildIdx; //rightChild == leftChild + 1
    int firstMeshIdx;
    int meshCount;
  };

  void filterMesh(std::vector<Object_ptr> &objects);
  void updateNodeBounds(int nodeIdx);
  void subDivide(int nodeIdx);

  std::vector<BVHNode> bvhNode;
  std::vector<Mesh *> meshes;
  const int rootNodeIdx = 0;
  int nodesUsed = 1;
  bool isInitialized_ = false;
};

// Inline Functions ------------------------------------------------------------
inline vec3 fmin(const vec3 &a, const vec3 &b) {
  return vec3(fmin(a[0], b[0]), fmin(a[1], b[1]), fmin(a[2], b[2]));
};

inline vec3 fmax(const vec3 &a, const vec3 &b) {
  return vec3(fmax(a[0], b[0]), fmax(a[1], b[1]), fmax(a[2], b[2]));
};
// -----------------------------------------------------------------------------
