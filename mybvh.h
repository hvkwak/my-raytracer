#include "utils/vec3.h"
#include "utils/Ray.h"
#include "utils/Material.h"
#include "utils/Triangle.h"
#include "utils/Data.h"
#include "Mesh.h"
#include <math.h>

class BVH {

public:

  BVH() = default;
  ~BVH();

  void init(std::vector<Mesh*> &meshes);
  void init_SoA(std::vector<Mesh*> &meshes, Data* data);
  bool isInitialized() const;
  bool intersectAABB(const Ray & ray, const vec3 & bb_min_, const vec3 & bb_max_) const;
  bool intersectBVH(const Ray &ray,
                    Material& intersection_material,
                    vec3 &intersection_point,
                    vec3 &intersection_normal,
                    double &intersection_distance,
                    int nodeIdx) const;
  bool intersectBVH_SoA(const Ray &ray,
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

  /// non-SoA
  void getData(std::vector<Mesh*> &meshes);
  void updateNodeBounds(int nodeIdx);
  void subdivide(int nodeIdx, int depth);
  void inplace_partition(int nodeIdx, double splitPos, int axis, int& i);

  /// SoA
  void updateNodeBounds_SoA(int nodeIdx);
  void subdivide_SoA(int nodeIdx, int depth);
  void inplace_partition_SoA(int nodeIdx, double splitPos, int axis, int & i);

  /// median Splitters
  double median(int axis, int nodeIdx);
  double median_SoA(int axis, int nodeIdx);
  double median_inplace(std::vector<double> &a);

  /// Nodes, Triangles, Meshes
  std::vector<BVHNode> bvhNodes_;
  std::vector<Triangle*> triangles_;
  std::vector<Mesh*> meshes_;

  int triCount = 0;
  int rootNodeIdx_ = 0;
  int nodesUsed_ = 1;
  bool isInitialized_ = false;

  Data* data_;
};

// Inline Functions ------------------------------------------------------------
inline vec3 fmin(const vec3 &a, const vec3 &b) {
  return vec3(fmin(a[0], b[0]), fmin(a[1], b[1]), fmin(a[2], b[2]));
};

inline vec3 fmax(const vec3 &a, const vec3 &b) {
  return vec3(fmax(a[0], b[0]), fmax(a[1], b[1]), fmax(a[2], b[2]));
};
// -----------------------------------------------------------------------------
