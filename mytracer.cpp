// ============================================================================
// Computer Graphics(Graphische Datenverarbeitung) - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#include "Raytracer.h"
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
  // reads scene and build BVH
  read_scene(filename);
  bvh.init(meshes_);
}

void Raytracer::init_cuda(const std::string &filename){
  // pre read scene, read scene, build SoA, and build BVH
  pre_read_scene(filename);
  read_scene(filename);
  build_Data();
  bvh.initSoA(meshes_, data_);
  init_device();
  prepareDeviceResources();
}


void Raytracer::init_device(void){
  // Initialize CUDA device
  int dev = 0;
  cudaDeviceProp deviceProp;
  CHECK(cudaGetDeviceProperties(&deviceProp, dev));
  printf("Using Device %d: %s\n", dev, deviceProp.name);
  CHECK(cudaSetDevice(dev));
}

void Raytracer::allocate(size_t new_bytes, size_t &old_bytes, void* &ptr){
  // (re)allocate only if size changed
  if (new_bytes != old_bytes) {
    if (ptr) {
      CHECK(cudaFree(ptr));
      ptr = nullptr;
      old_bytes = 0;
    }
    old_bytes = new_bytes;
    CHECK(cudaMalloc((&ptr), old_bytes));
    CHECK(cudaMemset(ptr, 0, old_bytes));
  }
}

void Raytracer::prepareDeviceResources() {

  // allocate nPixels
  size_t nPixels = static_cast<size_t>(camera_.width_) * camera_.height_;
  size_t pixels_bytes   = nPixels * sizeof(vec4);
  allocate(pixels_bytes, d_pixels_bytes_, reinterpret_cast<void*&>(d_pixels_)); // TODO

  // allocate Lights
  size_t nLights = lights_.size();
  size_t lights_bytes = nLights * sizeof(Light);
  allocate(lights_bytes, d_lights_bytes_, reinterpret_cast<void*&>(d_lightsPosition_));
  allocate(lights_bytes, d_lights_bytes_, reinterpret_cast<void*&>(d_lightsColor_));
  copyLights(); // from std::vector
  // CHECK(cudaMemcpy(d_A, h_A, nBytes, cudaMemcpyHostToDevice)); TODO: lights to array!
}

void Raytracer::copyLights(){
  int nLights = lights_.size();
  size_t nBytes = nLights * sizeof(vec4);
  vec4* lightPos = (vec4*)malloc(nBytes);
  vec4* lightColor = (vec4*)malloc(nBytes);
  for (int i = 0; i < nLights; i++){
    lightPos[i] = lights_[i].position;
    lightColor[i] = lights_[i].color;
  }
  CHECK(cudaMemcpy(d_lightsPosition_, lightPos, nBytes, cudaMemcpyHostToDevice));
  CHECK(cudaMemcpy(d_lightsColor_, lightColor, nBytes, cudaMemcpyHostToDevice));
  free(lightPos);
  free(lightColor);
}

void Raytracer::compute_image_cuda(){

  std::cout << "Raytracer::compute_image_cuda()..." ;

  // allocate memory by resizing image
  image_.resize(camera_.width_, camera_.height_);
  Image tmpimage;
  tmpimage.resize(camera_.width_, camera_.height_);

  // invoke kernel at host side
  size_t nPixels = static_cast<size_t>(camera_.width_) * camera_.height_;
  int nThreads = 32;
  dim3 block (nThreads, nThreads);
  dim3 grid ((nPixels+block.x-1)/block.x, (nPixels+block.y-1)/block.y);

  // TODO: compute_image_device<<<grid, block>>>


  // // synchronize GPU execution
  // CHECK(cudaDeviceSynchronize());

  // // copy result from device to host
  // CHECK(cudaMemcpy(image_.data(), dPixels, nBytesPixels, cudaMemcpyDeviceToHost));

  // // free device memory
  // CHECK(cudaFree(dPixels));
  // CHECK(cudaFree(dLights));init_cpu(filename);

  // std::cout << " done." << std::endl;
}

/**
 * @brief Build Structure-of-Arrays (SoA) data layout from mesh objects
 *
 * Transforms mesh data from AoS to SoA format for GPU-friendly memory access
 */
void Raytracer::build_Data(){

  std::cout << "build Data...";

  /// SoA memory allocation: Unified Memory!
  // per-vertex / indices / per-tri

  CHECK(cudaMallocManaged(&data_,                                             sizeof(Data)));
  CHECK(cudaMallocManaged(&data_->vertexPos_,         data_->tVertexCount_    * sizeof(vec4)));
  CHECK(cudaMallocManaged(&data_->vertexNormals_,     data_->tVertexCount_    * sizeof(vec4)));
  CHECK(cudaMallocManaged(&data_->vertexIdx_,         data_->tVertexIdxCount_ * sizeof(int)));
  CHECK(cudaMallocManaged(&data_->textureIdx_,        data_->tTextIdxCount_   * sizeof(int)));
  CHECK(cudaMallocManaged(&data_->textureCoordinatesU_, data_->tTextCoordCount_ * sizeof(double)));
  CHECK(cudaMallocManaged(&data_->textureCoordinatesV_, data_->tTextCoordCount_ * sizeof(double)));
  CHECK(cudaMallocManaged(&data_->normals_,           (data_->tVertexIdxCount_/3) * sizeof(vec4)));

  // per-vertex mesh ownership (ID instead of stMesh)
  CHECK(cudaMallocManaged(&data_->vertexMeshId_,      data_->tVertexCount_    * sizeof(int)));

  // per-mesh metadata
  CHECK(cudaMallocManaged(&data_->firstVertex_,       data_->tMeshCount_ * sizeof(int)));
  CHECK(cudaMallocManaged(&data_->vertexCount_,       data_->tMeshCount_ * sizeof(int)));
  CHECK(cudaMallocManaged(&data_->firstVertexIdx_,    data_->tMeshCount_ * sizeof(int)));
  CHECK(cudaMallocManaged(&data_->vertexIdxCount_,    data_->tMeshCount_ * sizeof(int)));
  CHECK(cudaMallocManaged(&data_->firstTextCoord_,    data_->tMeshCount_ * sizeof(int)));
  CHECK(cudaMallocManaged(&data_->textCoordCount_,    data_->tMeshCount_ * sizeof(int)));
  CHECK(cudaMallocManaged(&data_->firstTextIdx_,      data_->tMeshCount_ * sizeof(int)));
  CHECK(cudaMallocManaged(&data_->textIdxCount_,      data_->tMeshCount_ * sizeof(int)));

  // per-mesh texture tables
  /// count total texels
  for (Mesh* m : meshes_){
    data_->tTotalTexels_ += size_t(m->texture_.width()) * m->texture_.height();
  }
  CHECK(cudaMallocManaged(&data_->meshTexels_,        data_->tTotalTexels_ * sizeof(vec4)));
  CHECK(cudaMallocManaged(&data_->meshTexWidth_,      (size_t)data_->tMeshCount_ * sizeof(int)));
  CHECK(cudaMallocManaged(&data_->meshTexHeight_,     (size_t)data_->tMeshCount_ * sizeof(int)));
  CHECK(cudaMallocManaged(&data_->meshTexOffset_,     (size_t)data_->tMeshCount_ * sizeof(size_t)));

  // Material per Mesh
  CHECK(cudaMallocManaged(&data_->materialAmbient_,    data_->tMeshCount_ * sizeof(vec4)));
  CHECK(cudaMallocManaged(&data_->materialDiffuse_,    data_->tMeshCount_ * sizeof(vec4)));
  CHECK(cudaMallocManaged(&data_->materialSpecular_,   data_->tMeshCount_ * sizeof(vec4)));
  CHECK(cudaMallocManaged(&data_->materialMirror_,     data_->tMeshCount_ * sizeof(double)));
  CHECK(cudaMallocManaged(&data_->materialShininess_,  data_->tMeshCount_ * sizeof(double)));
  CHECK(cudaMallocManaged(&data_->materialShadowable_, data_->tMeshCount_ * sizeof(bool)));

  /// Data copy per Mesh
  int meshIdx = 0;
  size_t meshTexOffset = 0;
  int vbase   = 0; // vertex base
  int tbase   = 0; // texture base
  int ibase   = 0; // index
  for (Mesh* mesh : meshes_){
    /// Copy vertex data
    int vertexCount = mesh->vertices_.size();
    int count = 0;
    for (Vertex vertex : mesh->vertices_){
      data_->vertexPos_[vbase+count] = vertex.position;
      data_->vertexNormals_[vbase+count] = vertex.normal;
      data_->vertexMeshId_[vbase+count] = meshIdx;
      count++;
    }
    data_->firstVertex_[meshIdx] = vbase;
    data_->vertexCount_[meshIdx] = vertexCount;

    /// Copy texture coordinates
    int textureCount = mesh->u_coordinates_.size();
    for (int t = 0; t < textureCount; t++){
      data_->textureCoordinatesU_[tbase + t] = mesh->u_coordinates_[t];
      data_->textureCoordinatesV_[tbase + t] = mesh->v_coordinates_[t];
    }
    data_->firstTextCoord_[meshIdx] = tbase;
    data_->textCoordCount_[meshIdx] = textureCount;

    /// Copy triangle indices
    count = 0;
    for (Triangle triangle : mesh->triangles_){
      data_->vertexIdx_[ibase+3*count] = (vbase + triangle.i0);
      data_->vertexIdx_[ibase+3*count+1] = (vbase + triangle.i1);
      data_->vertexIdx_[ibase+3*count+2] = (vbase + triangle.i2);
      data_->textureIdx_[ibase+3*count] = (tbase + triangle.iuv0);
      data_->textureIdx_[ibase+3*count+1] = (tbase + triangle.iuv1);
      data_->textureIdx_[ibase+3*count+2] = (tbase + triangle.iuv2);
      data_->normals_[ibase/3 + count] = (triangle.normal);
      count++;
    }
    data_->firstVertexIdx_[meshIdx] = ibase;
    data_->vertexIdxCount_[meshIdx] = count*3;
    data_->firstTextIdx_[meshIdx] = ibase;
    data_->textIdxCount_[meshIdx] = count*3;

    /// per-mesh texture upload into UM (one block per mesh)
    if (mesh->hasTexture_){
      const int W = mesh->texture_.width();
      const int H = mesh->texture_.height();
      data_->meshTexWidth_[meshIdx] = W;
      data_->meshTexHeight_[meshIdx] = H;
      data_->meshTexOffset_[meshIdx] = meshTexOffset;
      for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
          data_->meshTexels_[meshTexOffset + y * size_t(W) + x] = mesh->texture_(x, y);
      meshTexOffset += size_t(W) * H;
    }else{
      data_->meshTexWidth_[meshIdx] = -1;
      data_->meshTexHeight_[meshIdx] = -1;
      data_->meshTexOffset_[meshIdx] = size_t(-1);
    }

    // add Material
    data_->materialAmbient_[meshIdx] = mesh->material_.ambient;
    data_->materialDiffuse_[meshIdx] = mesh->material_.diffuse;
    data_->materialSpecular_[meshIdx] = mesh->material_.specular;
    data_->materialMirror_[meshIdx] = mesh->material_.mirror;
    data_->materialShininess_[meshIdx] = mesh->material_.shininess;
    data_->materialShadowable_[meshIdx] = mesh->material_.shadowable;

    // base increment
    vbase = vbase + vertexCount;
    ibase = ibase + count*3;
    tbase = tbase + textureCount;
    meshIdx++;
  }
  std::cout << " done. \n" << std::flush;
}


/**
 * @brief pre-reads scene to build SoA
 */
void Raytracer::pre_read_scene(const std::string &filename)
{
  // Data(concat Vectors) clean up
  freeData();

  /// Pre-read variables reset
  data_->tMeshCount_ = 0;
  data_->tVertexCount_ = 0;
  data_->tVertexIdxCount_ = 0;
  data_->tTextCoordCount_ = 0;
  data_->tTextIdxCount_ = 0;

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

      // per read object will be pre-read variables are updated
      pre_read_obj(fn.c_str());
      data_->tMeshCount_++;
    } else {
      continue;
    }
  }
  ifs.close();
}

void Raytracer::freeVariables() {
  if (d_pixels_) {
    CHECK(cudaFree(d_pixels_));
    d_pixels_ = nullptr;
    d_pixels_bytes_ = 0;
  }
  if (d_lightsPosition_) {
    CHECK(cudaFree(d_lightsPosition_));
    d_lightsPosition_ = nullptr;
    d_lights_bytes_ = 0;
  }
  if (d_lightsColor_) {
    CHECK(cudaFree(d_lightsColor_));
    d_lightsColor_ = nullptr;
    d_lights_bytes_ = 0;
  }
}

void Raytracer::freeData(){
  auto F = [&](auto*& p){ if (p) { cudaFree(p); p = nullptr; } };
  F(data_->vertexPos_);
  F(data_->vertexNormals_);
  F(data_->vertexIdx_);
  F(data_->textureIdx_);
  F(data_->textureCoordinatesU_);
  F(data_->textureCoordinatesV_);
  F(data_->firstVertex_);
  F(data_->vertexCount_);
  F(data_->firstVertexIdx_);
  F(data_->vertexIdxCount_);
  F(data_->firstTextCoord_);
  F(data_->textCoordCount_);
  F(data_->firstTextIdx_);
  F(data_->textIdxCount_);
  F(data_->normals_);
  F(data_->vertexMeshId_);
  F(data_->meshTexWidth_);
  F(data_->meshTexHeight_);
  F(data_->meshTexels_);
  F(data_->meshTexWidth_);
  F(data_->meshTexHeight_);
  F(data_->meshTexOffset_);
  F(data_->materialAmbient_);
  F(data_->materialDiffuse_);
  F(data_->materialSpecular_);
  F(data_->materialMirror_);
  F(data_->materialShininess_);
  F(data_->materialShadowable_);
  F(data_);
}

Raytracer::~Raytracer(){
  freeVariables();
  freeData();
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
      /// too expensive to read the file for Raytracer!
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
  data_->tVertexCount_ = data_->tVertexCount_ + vertexCount;
  data_->tVertexIdxCount_ = data_->tVertexIdxCount_ + vertexIdxCount;
  data_->tTextCoordCount_ = data_->tTextCoordCount_ + textCoordCount;
  data_->tTextIdxCount_ = data_->tTextIdxCount_ + textIdxCount;

  std::cout << "\n  pre-read " << _filename << ": "
            << data_->tVertexCount_ << " vertices, "
            << data_->tVertexIdxCount_/3 << " triangles, "
            << data_->tTextCoordCount_ << " textCoordCount, "
            << data_->tTextIdxCount_ << " textIdxCount"
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
