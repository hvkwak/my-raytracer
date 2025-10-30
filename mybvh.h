#include "utils/vec3.h"
#include "utils/Ray.h"
#include "utils/Material.h"
#include "utils/Triangle.h"
#include "Mesh.h"
#include <math.h>

class BVH {

public:

  BVH() = default;
  ~BVH();
  void init(std::vector<Mesh*> &meshes, int triCount);
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
    int firstTriIdx, triCount;
  };

  void getData(std::vector<Mesh*> &meshes);
  void updateNodeBounds(int nodeIdx);
  void subdivide(int nodeIdx, int depth);
  double median(int axis, int firstTriIdx, int triCount);
  double median_inplace(std::vector<double> &a);

  std::vector<BVHNode> bvhNodes;
  std::vector<Triangle*> triangles;
  std::vector<Triangle*> triangles;
  std::vector<Mesh*> meshes;

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
