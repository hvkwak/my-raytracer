// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#include "Raytracer.h"
#include "utils/vec4.h"
#include "utils/vec4.h"
#include "utils/Material.h"
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include "common/common.h"
#include <cuda_runtime.h>
#include "mytracer_gpu.h"

void Raytracer::init_cpu(const std::string &filename){
  read_scene(filename);
  bvh.init(meshes_);
}
void Raytracer::init_cuda(const std::string &filename){
  pre_read_scene(filename);
  read_scene(filename);
  buildSoA();
  bvh.initSoA(meshes_, &data_);
}

// void Raytracer::cudaInit(void){
//   // Initialize CUDA device
//   int dev = 0;
//   cudaDeviceProp deviceProp;
//   CHECK(cudaGetDeviceProperties(&deviceProp, dev));
//   printf("Using Device %d: %s\n", dev, deviceProp.name);
//   CHECK(cudaSetDevice(dev));
// }


void Raytracer::compute_image_cuda(){
  std::cout << "Raytracer::compute_image_cuda()..." ;

  // allocate memory by resizing image
  image_.resize(camera_.width_, camera_.height_);
  Image tmpimage;
  tmpimage.resize(camera_.width_, camera_.height_);

  // malloc device memory for image pixels
  int nPixels = camera_.width_*camera_.height_;
  size_t nBytesPixels = nPixels*sizeof(vec4);
  vec4 *dPixels;
  CHECK(cudaMalloc(&dPixels, nBytesPixels));

  // malloc device memory for lights and copy from host
  int numLights = lights_.size();
  Light* dLights;
  CHECK(cudaMalloc(&dLights, numLights * sizeof(Light)));
  CHECK(cudaMemcpy(dLights, lights_.data(), numLights * sizeof(Light), cudaMemcpyHostToDevice));

  // invoke kernel at host side
  int nThreads = 32;
  dim3 block (nThreads, nThreads);
  dim3 grid ((nPixels+block.x-1)/block.x, (nPixels+block.y-1)/block.y);

  // traceOnGPU<<<grid, block>>>(dPixels, camera_.width_, camera_.height_, camera_, dLights, numLights, data_,
  //                             getBackground(),
  //                             getAmbience(),
  //                             getMaxDepth());

  // // synchronize GPU execution
  // CHECK(cudaDeviceSynchronize());

  // // copy result from device to host
  // CHECK(cudaMemcpy(image_.data(), dPixels, nBytesPixels, cudaMemcpyDeviceToHost));

  // // free device memory
  // CHECK(cudaFree(dPixels));
  // CHECK(cudaFree(dLights));

  // std::cout << " done." << std::endl;
}

/**
 * @brief Build Structure-of-Arrays (SoA) data layout from mesh objects
 *
 * Transforms mesh data from AoS to SoA format for GPU-friendly memory access
 */
void Raytracer::buildSoA(){

  std::cout << "buildSoA...";

  /// SoA memory allocation
  data_.meshes_ = (stMesh*)malloc(vertexCount_*sizeof(stMesh));
  data_.vertexPos_ = (vec4*)malloc(vertexCount_*sizeof(vec4));
  data_.vertexIdx_ = (int*)malloc(vertexIdxCount_*sizeof(int));
  data_.textureCoordinatesU_ = (double*)malloc(textCoordCount_*sizeof(double));
  data_.textureCoordinatesV_ = (double*)malloc(textCoordCount_*sizeof(double));
  data_.textureIdx_ = (int*)malloc(textIdxCount_*sizeof(int));
  data_.firstVertex_ = (int*)malloc(meshCount_*sizeof(int));
  data_.vertexCount_ = (int*)malloc(meshCount_*sizeof(int));
  data_.firstVertexIdx_ = (int*)malloc(meshCount_*sizeof(int));
  data_.vertexIdxCount_ = (int*)malloc(meshCount_*sizeof(int));
  data_.firstTextCoord_ = (int*)malloc(meshCount_*sizeof(int));
  data_.textCoordCount_ = (int*)malloc(meshCount_*sizeof(int));
  data_.firstTextIdx_ = (int*)malloc(meshCount_*sizeof(int));
  data_.textIdxCount_ = (int*)malloc(meshCount_*sizeof(int));
  data_.normals_ = (vec4*)malloc(vertexIdxCount_/3*sizeof(vec4)); // triangle
  data_.vertexNormals_ = (vec4*)malloc(vertexCount_*sizeof(vec4));

  for (Mesh* mesh : meshes_){
    // Copy vertex data
    int vbase = data_.vertexPos_.size();
    int vertexCount = mesh->vertices_.size();
    for (Vertex vertex : mesh->vertices_){
      data_.vertexPos_.push_back(vertex.position);
      data_.vertexNormals_.push_back(vertex.normal);
    }
    data_.firstVertex_.push_back(vbase);
    data_.vertexCount_.push_back(vertexCount);
    data_.meshes_.insert(data_.meshes_.end(), vertexCount, mesh);

    // Copy texture coordinates
    int tbase = data_.textureCoordinatesU_.size();
    int textureCount = mesh->u_coordinates_.size();
    data_.textureCoordinatesU_.insert(data_.textureCoordinatesU_.end(),
                                      mesh->u_coordinates_.begin(), mesh->u_coordinates_.end());
    data_.textureCoordinatesV_.insert(data_.textureCoordinatesV_.end(),
                                      mesh->v_coordinates_.begin(), mesh->v_coordinates_.end());
    data_.firstTextIdx_.push_back(tbase);
    data_.textIdxCount_.push_back(textureCount);

    // Copy triangle indices
    int ibase = data_.vertexIdx_.size();
    int vertexIdxCount = mesh->triangles_.size() * 3;
    for (Triangle triangle : mesh->triangles_){
      data_.vertexIdx_.push_back(vbase + triangle.i0);
      data_.vertexIdx_.push_back(vbase + triangle.i1);
      data_.vertexIdx_.push_back(vbase + triangle.i2);
      data_.textureIdx_.push_back(tbase + triangle.iuv0);
      data_.textureIdx_.push_back(tbase + triangle.iuv1);
      data_.textureIdx_.push_back(tbase + triangle.iuv2);
      data_.normals_.push_back(triangle.normal);
    }
    data_.firstVertexIdx_.push_back(ibase);
    data_.vertexIdxCount_.push_back(vertexIdxCount);
    data_.firstTextIdx_.push_back(ibase);
    data_.textIdxCount_.push_back(vertexIdxCount);
  }
  std::cout << " done. \n" << std::flush;
}


/**
 * @brief pre-reads scene to build SoA
 */
void Raytracer::pre_read_scene(const std::string &filename)
{
  // // Data(concat Vectors) clean up
  if (data_.meshes_ != nullptr) free(data_.meshes_);
  if (data_.vertexPos_ != nullptr) free(data_.vertexPos_);
  if (data_.vertexIdx_ != nullptr) free(data_.vertexIdx_);
  if (data_.textureCoordinatesU_ != nullptr) free(data_.textureCoordinatesU_);
  if (data_.textureCoordinatesV_ != nullptr) free(data_.textureCoordinatesV_);
  if (data_.textureIdx_ != nullptr) free(data_.textureIdx_);
  if (data_.firstVertex_ != nullptr) free(data_.firstVertex_);
  if (data_.vertexCount_ != nullptr) free(data_.vertexCount_);
  if (data_.firstVertexIdx_ != nullptr) free(data_.firstVertexIdx_);
  if (data_.vertexIdxCount_ != nullptr) free(data_.vertexIdxCount_);
  if (data_.firstTextCoord_ != nullptr) free(data_.firstTextCoord_);
  if (data_.textCoordCount_ != nullptr) free(data_.textCoordCount_);
  if (data_.firstTextIdx_ != nullptr) free(data_.firstTextIdx_);
  if (data_.textIdxCount_ != nullptr) free(data_.textIdxCount_);
  if (data_.normals_ != nullptr) free(data_.normals_);
  if (data_.vertexNormals_ != nullptr) free(data_.vertexNormals_);

  // data_.meshes_.clear();
  // data_.vertexPos_.clear();
  // data_.vertexIdx_.clear();
  // data_.textureCoordinatesU_.clear();
  // data_.textureCoordinatesV_.clear();
  // data_.textureIdx_.clear();
  // data_.firstVertex_.clear();
  // data_.vertexCount_.clear();
  // data_.firstVertexIdx_.clear();
  // data_.vertexIdxCount_.clear();
  // data_.firstTextCoord_.clear();
  // data_.textCoordCount_.clear();
  // data_.firstTextIdx_.clear();
  // data_.textIdxCount_.clear();
  // data_.normals_.clear();
  // data_.vertexNormals_.clear();

  data_.tMeshCount = 0;
  data_.tVertexCount_ = 0;
  data_.tVertexIdxCount_ = 0;
  data_.tTextCoordCount_ = 0;
  data_.tTextIdxCount_ = 0;

  std::ifstream ifs(filename);
  if (!ifs) {
    std::cerr << "Cannot open file " << filename << std::endl;
    exit(1);
  }

  char line[200];
  std::string token;

  // parse file
  while (ifs && (ifs >> token) && (!ifs.eof())) {
    if (token[0] == '#') {
      ifs.getline(line, 200);
    } else if (token == "mesh") {
      std::string fn, mode;
      ifs >> fn >> mode;

      // add path of scene-file to mesh's filename
      std::string path(filename);
      path = path.substr(0, path.find_last_of('/') + 1);
      fn = path + fn;
      pre_read_obj(fn.c_str());
      meshCount_++;
    } else {
      continue;
    }
  }
  ifs.close();

  /// total mesh count
  data_.tMeshCount = meshCount_;
}

Raytracer::~Raytracer(){
  free(data_.vertexPos_);
  free(data_.vertexIdx_);
  free(data_.textureCoordinatesU_);
  free(data_.textureCoordinatesV_);
  free(data_.textureIdx_);
  free(data_.firstVertex_);
  free(data_.vertexCount_);
  free(data_.firstVertexIdx_);
  free(data_.vertexIdxCount_);
  free(data_.firstTextCoord_);
  free(data_.textCoordCount_);
  free(data_.firstTextIdx_);
  free(data_.textIdxCount_);
  free(data_.normals_);
  free(data_.vertexNormals_);
}

/**
 * @brief pre reads obj file to enable memory allocation @ pre-read-scene
 *        (to know how many vertices and indices exist)
 */
void Raytracer::pre_read_obj(const char* _filename)
{
  int vertexCount = 0;
  int vertexIdxCount = 0;
  int textCoordCount = 0;
  int textIdxCount = 0;

  // open obj file
  std::ifstream ifs(_filename);
  if (!ifs) {
    std::cerr << "Can't open " << _filename << "\n";
    return;
  }

  bool hasNormals = false;
  bool hasUV = false;

  std::string filename(_filename);
  std::string line;
  int counter = -1;
  std::map<int, bool> uvDone;
  std::vector<Image> textures;
  // parse line by line
  while (std::getline(ifs, line)) {
    // divide line into header (first word) and lineData (rest)
    size_t firstSpace = line.find_first_of(" ");
    std::string header = line.substr(0, firstSpace);
    std::istringstream lineData(line.substr(firstSpace + 1));

    // vertices
    if (header == "v") {
      vertexCount++;
      continue;
    }

    // uv-coordinates
    if (header == "vt") {
      hasUV = true;
      textCoordCount++;
      continue;
    }

    if (header == "vn") {
      hasNormals = true;
      continue;
    }

    // material file
    if (header == "mtllib") {
      continue;
    }

    // start of new material
    if (header == "usemtl") {
      continue;
    }

    // faces
    if (header == "f") {
      textIdxCount = textIdxCount + 3;
      vertexIdxCount = vertexIdxCount + 3;
      continue;
    }
  }
  data_.tVertexCount_ = data_.tVertexCount_ + vertexCount;
  data_.tVertexIdxCount_ = data_.tVertexIdxCount_ + vertexIdxCount;
  data_.tTextCoordCount_ = data_.tTextCoordCount_ + textCoordCount;
  data_.tTextIdxCount_ = data_.tTextIdxCount_ + textIdxCount;

  std::cout << "\n  pre-read " << _filename << ": "
            << data_.tVertexCount << " vertices, "
            << data_.tVertexIdxCount/3 << " triangles, "
            << data_.tTextCoordCount << " textCoordCount, "
            << data_.tTextIdxCount << " textIdxCount"
            << std::flush;
}

/**
 * @brief returns diffuse term
 *
 * @param point the point, of which the color should be determined
 * @param normal normal at the point
 * @param light light source
 */
double Raytracer::diffuse(const vec4 &point, const vec4 &normal, const Light &light) const {
  vec4 ray_from_point_to_light = normalize(light.position - point);
  double cosTheta = dot(normal, ray_from_point_to_light);
  cosTheta = std::max(0.0, cosTheta);
  return cosTheta;
}

/**
 * @brief returns specular reflection term
 *
 * @param point the point, of which the color should be dtermined
 * @param normal normal at the point
 * @param view normalized direction from the 'point' to the viewer's position
 */
double Raytracer::reflection(const vec4 &point, const vec4 &normal, const vec4 &view, const Light &light) const {
  if (diffuse(point, normal, light) > 0.0){
    vec4 ray_from_point_to_light = normalize(light.position - point);
    vec4 ray_reflected = normalize(mirror(ray_from_point_to_light, normal));
    double cosTheta = dot(ray_reflected, view);
    cosTheta = std::max(0.0, cosTheta);
    return cosTheta;
  } else {
    return 0.0;
  }
}

/**
 * @brief Compute reflected ray contribution (recursive raytracing)
 *
 * @param ray Incoming ray
 * @param material Material of the intersected object
 * @param point Intersection point
 * @param normal Surface normal at intersection
 * @param depth Current recursion depth
 * @return Reflected color contribution
 */
vec4 Raytracer::subtrace(const Ray &ray, const Material &material, const vec4 &point, const vec4 &normal, const int depth){
  if (material.mirror > 0.0){
    // Generate reflected ray
    vec4 v = reflect(ray.direction_, normal);
    const double epsilon = 1e-4;  // Offset to avoid self-intersection
    Ray reflected_ray = Ray(point + epsilon * v, v);
    return material.mirror * trace(reflected_ray, depth + 1);
  }
  return {0, 0, 0};
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
vec4 Raytracer::lighting(const vec4 &point, const vec4 &normal,
                         const vec4 &view, const Material &material) const {
  const double epsilon = 1e-4;  // Offset to avoid self-intersection
  vec4 color(0.0, 0.0, 0.0);

  // Ambient component (approximates global illumination)
  color[0] += ambience_[0] * material.ambient[0];
  color[1] += ambience_[1] * material.ambient[1];
  color[2] += ambience_[2] * material.ambient[2];

  // Process each light source
  for (const Light &light : lights_) {
    // Compute diffuse and specular components
    double diffuse_ = diffuse(point, normal, light);
    double dot_rv = reflection(point, normal, view, light);
    double reflection_ = dot_rv;
    // Compute specular exponent
    double i = 0.0;
    while (i < material.shininess - 1){
      reflection_ *= dot_rv;
      i += 1.0;
    }

    // Shadow calculation
    bool isShadow = false;
    if (material.shadowable){
      Material shadow_material;
      vec4 shadow_point;
      vec4 shadow_normal;
      double shadow_t;
      vec4 light_direction = normalize(light.position - point);
      double light_distance = norm(light.position - point);
      Ray shadow_ray = Ray(point + epsilon * light_direction, light_direction);
      bool isIntersect = intersect_scene(shadow_ray, shadow_material,
                                         shadow_point, shadow_normal, shadow_t);
      isShadow = isIntersect && shadow_t < light_distance && 0.0 < shadow_t;
    }

    // Accumulate light contribution if not in shadow
    color[0] += light.color[0] * !isShadow * (material.diffuse[0] * diffuse_ + material.specular[0] * reflection_);
    color[1] += light.color[1] * !isShadow * (material.diffuse[1] * diffuse_ + material.specular[1] * reflection_);
    color[2] += light.color[2] * !isShadow * (material.diffuse[2] * diffuse_ + material.specular[2] * reflection_);
  }
  return color;
}
