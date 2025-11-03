// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#include "Raytracer.h"
#include "utils/vec3.h"
#include "utils/Material.h"
#include <fstream>
#include <sstream>
#include <string>
#include <map>

/**
 * @brief builds SoA based on the read meshes
 */
void Raytracer::buildSoA(){
  std::cout << "buildSoA...";
  for (Mesh* mesh : meshes_){

    // Vertex
    int vbase = data_.vertexPos_.size();
    int vertexCount = mesh->vertices_.size();
    for (Vertex vertex : mesh->vertices_){
      data_.vertexPos_.push_back(vertex.position); // :(
    }
    // data_.vertexPos_.insert(data_.vertexPos_.end(), mesh->vertices_.begin(), mesh->vertices_.end());
    data_.firstVertex_.push_back(vbase);
    data_.vertexCount_.push_back(vertexCount);

    // Vertex Indicies + Texture Indices
    int ibase = data_.vertexIdx_.size();
    int vertexIdxCount = mesh->triangles_.size()*3;
    for (Triangle triangle : mesh->triangles_){
      data_.vertexIdx_.push_back(triangle.i0);
      data_.vertexIdx_.push_back(triangle.i1);
      data_.vertexIdx_.push_back(triangle.i2);
      data_.textureIdx_.push_back(triangle.iuv0);
      data_.textureIdx_.push_back(triangle.iuv1);
      data_.textureIdx_.push_back(triangle.iuv2);
    }
    data_.firstVertexIdx_.push_back(ibase);
    data_.vertexIdxCount_.push_back(vertexIdxCount);
    data_.firstTextIdx_.push_back(ibase);
    data_.textIdxCount_.push_back(vertexIdxCount);

    // Meshes
    data_.meshes_.insert(data_.meshes_.end(), vertexIdxCount, mesh);

    // Texture Coordinates
    int tbase = data_.textureCoordinatesU_.size();
    int textureCount = mesh->u_coordinates_.size(); // same as V
    data_.textureCoordinatesU_.insert(data_.textureCoordinatesU_.end(), mesh->u_coordinates_.begin(), mesh->u_coordinates_.end());
    data_.textureCoordinatesV_.insert(data_.textureCoordinatesV_.end(), mesh->v_coordinates_.begin(), mesh->v_coordinates_.end());
    data_.firstTextIdx_.push_back(tbase);
    data_.textIdxCount_.push_back(textureCount);
  }
  std::cout << " done. \n" << std::flush;
}


/**
 * @brief pre-reads scene to build SoA
 */
void Raytracer::pre_read_scene(const std::string &filename)
{
  // Data(concat Vectors) clean up
  data_.meshes_.clear();
  data_.vertexPos_.clear();
  data_.vertexIdx_.clear();
  data_.textureCoordinatesU_.clear();
  data_.textureCoordinatesV_.clear();
  data_.textureIdx_.clear();

  data_.firstVertex_.clear();
  data_.vertexCount_.clear();
  data_.firstVertexIdx_.clear();
  data_.vertexIdxCount_.clear();
  data_.firstTextCoord_.clear();
  data_.textCoordCount_.clear();
  data_.firstTextIdx_.clear();
  data_.textIdxCount_.clear();

  meshCount_ = 0;
  vertexCount_ = 0;
  vertexIdxCount_ = 0;
  textCoordCount_ = 0;
  textIdxCount_ = 0;
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

  /// SoA memory allocation
  data_.meshes_.reserve(vertexIdxCount_);
  data_.vertexPos_.reserve(vertexCount_);
  data_.vertexIdx_.reserve(vertexIdxCount_);
  data_.textureCoordinatesU_.reserve(textCoordCount_);
  data_.textureCoordinatesV_.reserve(textCoordCount_);
  data_.textureIdx_.reserve(textIdxCount_);
  data_.firstVertex_.reserve(meshCount_);
  data_.vertexCount_.reserve(meshCount_);
  data_.firstVertexIdx_.reserve(meshCount_);
  data_.vertexIdxCount_.reserve(meshCount_);
  data_.firstTextCoord_.reserve(meshCount_);
  data_.textCoordCount_.reserve(meshCount_);
  data_.firstTextIdx_.reserve(meshCount_);
  data_.textIdxCount_.reserve(meshCount_);
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
  vertexCount_ = vertexCount_ + vertexCount;
  vertexIdxCount_ = vertexIdxCount_ + vertexIdxCount;
  textCoordCount_ = textCoordCount_ + textCoordCount;
  textIdxCount_ = textIdxCount_ + textIdxCount;

  std::cout << "\n  read " << _filename << ": "
            << vertexCount << " vertices, "
            << vertexIdxCount/3 << " triangles, "
            << textCoordCount << " textCoordCount, "
            << textIdxCount << " textIdxCount"
            << std::flush;
}

/**
 * @brief returns diffuse term
 *
 * @param point the point, of which the color should be determined
 * @param normal normal at the point
 * @param light light source
 */
double Raytracer::diffuse(const vec3 &point, const vec3 &normal, const Light &light) const {
  vec3 ray_from_point_to_light = normalize(light.position - point);
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
double Raytracer::reflection(const vec3 &point, const vec3 &normal, const vec3 &view, const Light &light) const {
  if (diffuse(point, normal, light) > 0.0){
    vec3 ray_from_point_to_light = normalize(light.position - point);
    vec3 ray_reflected = normalize(mirror(ray_from_point_to_light, normal));
    double cosTheta = dot(ray_reflected, view);
    cosTheta = std::max(0.0, cosTheta);
    return cosTheta;
  } else {
    return 0.0;
  }
}

/**
 * @brief recursive trace()
 *
 * @param ray: incoming ray for recursive trace()
 *        material: material of the object
 *        point: the point, of which the color should be determined
 *        normal: normal at the point
 * @return sub-raytraced color at the point
 */
vec3 Raytracer::subtrace(const Ray &ray, const Material &material, const vec3 &point, const vec3 &normal, const int depth){

  /** \todo in trace() @ Raytracer.cpp */
  if (material.mirror > 0.0){
    // generate reflected ray using normal, point, and ray.direction
    // vec3 v = mirror(-ray.direction_, normal);
    vec3 v = reflect(ray.direction_, normal);
    const double epsilon = 1e-4; // small offset to avoid self-intersection
    Ray ray_ = Ray(point + epsilon * v, v);
    return material.mirror * trace(ray_, depth + 1);
  }
  return {0, 0, 0};
}

/**
 * @brief Computes Phong lighting model (ambient + diffuse + specular) at a
 *        surface point (local illumination)
 * @param point: intersection point
 *        normal: intersection normal
 *        view: ray from intersection point to pixel
 *        material: material of intersection point
 *
 * @return RGB color/radiance of all light sources at point
 */
vec3 Raytracer::lighting(const vec3 &point, const vec3 &normal,
                         const vec3 &view, const Material &material) const {
  const double epsilon = 1e-4;  // small offset to avoid self-intersection
  vec3 color(0.0, 0.0, 0.0);

  // ambient: uniform in, uniform out. approximates global light
  // transport/exchange.
  // note that ambience_ is ambient light.
  color[0] += ambience_[0] * material.ambient[0]; // R
  color[1] += ambience_[1] * material.ambient[1]; // G
  color[2] += ambience_[2] * material.ambient[2]; // B

  // diffuse: direct in, uniform out. dull / mat surfaces
  // specular refelction: directed in, directed out, shiny surfaces.
  for (const Light &light : lights_) {

    // compute diffuse and specular term
    double diffuse_ = diffuse(point, normal, light);
    double dot_rv = reflection(point, normal, view, light);
    double reflection_ = dot_rv;
    double i = 0.0;
    while (i < material.shininess-1 ){
      reflection_ *= dot_rv;
      i += 1.0;
    }

    bool isShadow = false;
    if (material.shadowable){
      // compute hard shadow term
      Material shadow_material;
      vec3 shadow_point;
      vec3 shadow_normal;
      double shadow_t;
      vec3 light_direction = normalize(light.position - point);
      double light_distance = norm(light.position - point);
      Ray shadow_ray = Ray(point + epsilon * light_direction, light_direction);
      bool isIntersect = intersect_scene(shadow_ray, shadow_material,
                                         shadow_point, shadow_normal, shadow_t);
      isShadow = isIntersect && shadow_t < light_distance && 0.0 < shadow_t;
    }
    color[0] += light.color[0] * !isShadow * (material.diffuse[0] * diffuse_ + material.specular[0] * reflection_);
    color[1] += light.color[1] * !isShadow * (material.diffuse[1] * diffuse_ + material.specular[1] * reflection_);
    color[2] += light.color[2] * !isShadow * (material.diffuse[2] * diffuse_ + material.specular[2] * reflection_);
  }
  return color;
}

//=============================================================================
