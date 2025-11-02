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


// /**
//  * @brief read vertices and indices in concat vectors
//  *        (complete read in Mesh.cpp)
//  */
// bool Raytracer::read_obj(const char *_filename, Mesh* mesh)
// {
//   int vbase = data_.vVertexPos_.size();
//   int ibase = data_.vVertexIdx_.size();
//   data_.vFirstVertex_.push_back(vbase);
//   data_.vFirstIdx_.push_back(ibase);

//   int vertexCount = 0;
//   int triangleCount = 0;

//   // open obj file
//   std::ifstream ifs(_filename);
//   if (!ifs) {
//     std::cerr << "Can't open " << _filename << "\n";
//     return false;
//   }

//   bool hasNormals = false;
//   bool hasUV = false;

//   std::string filename(_filename);
//   std::string line;
//   // parse line by line
//   while (std::getline(ifs, line)) {
//     // divide line into header (first word) and lineData (rest)
//     size_t firstSpace = line.find_first_of(" ");
//     std::string header = line.substr(0, firstSpace);
//     std::istringstream lineData(line.substr(firstSpace + 1));

//     // vertices
//     if (header == "v") {
//       vec3 v;
//       lineData >> v[0] >> v[1] >> v[2];
//       data_.vVertexPos_.push_back(v);
//       vertexCount++;
//       continue;
//     }

//     // uv-coordinates
//     if (header == "vt") {
//       hasUV = true;
//       continue;
//     }

//     if (header == "vn") {
//       hasNormals = true;
//       continue;
//     }

//     // material file
//     if (header == "mtllib") {
//       continue;
//     }

//     // start of new material
//     if (header == "usemtl") {
//       continue;
//     }

//     // faces
//     if (header == "f") {
//       Triangle t;

//       int uv[3];

//       enum { NORMALS, UV, BOTH, NONE } nuv_status;
//       if (hasUV)
//         nuv_status = hasNormals ? BOTH : UV;
//       else
//         nuv_status = hasNormals ? NORMALS : NONE;

//       // dummy variables for / and normal indices
//       int d1;
//       char d2;

//       // read in face indices and uv indices, skip normal indices
//       switch (nuv_status) {
//       case BOTH:
//         // format: index0/texture0/normal0 index1/texture1/normal1
//         // index2/texture2/normal2
//         lineData >> t.i0 >> d2 >> uv[0] >> d2 >> d1;
//         lineData >> t.i1 >> d2 >> uv[1] >> d2 >> d1;
//         lineData >> t.i2 >> d2 >> uv[2] >> d2 >> d1;

//         uv[0]--;
//         uv[1]--;
//         uv[2]--;
//         t.iuv0 = uv[0];
//         t.iuv1 = uv[1];
//         t.iuv2 = uv[2];
//         break;
//       case UV:
//         // format: index0/texture0 index1/texture1 index2/texture2
//         lineData >> t.i0 >> d2 >> uv[0];
//         lineData >> t.i1 >> d2 >> uv[1];
//         lineData >> t.i2 >> d2 >> uv[2];

//         uv[0]--;
//         uv[1]--;
//         uv[2]--;
//         t.iuv0 = uv[0];
//         t.iuv1 = uv[1];
//         t.iuv2 = uv[2];
//       case NORMALS:
//         // format: index0//normal0 index1//normal1 index2//normal2
//         lineData >> t.i0 >> d2 >> d2 >> d1;
//         lineData >> t.i1 >> d2 >> d2 >> d1;
//         lineData >> t.i2 >> d2 >> d2 >> d1;
//       case NONE:
//         // format: index0 index1 index2
//         lineData >> t.i0 >> t.i1 >> t.i2;
//       }

//       // decrement because obj indices start by 1
//       t.i0--;
//       t.i1--;
//       t.i2--;

//       data_.vVertexIdx_.push_back(vbase + t.i0);
//       data_.vVertexIdx_.push_back(vbase + t.i1);
//       data_.vVertexIdx_.push_back(vbase + t.i2);
//       data_.vMeshesPerIdx_.push_back(mesh);
//       data_.vMeshesPerIdx_.push_back(mesh);
//       data_.vMeshesPerIdx_.push_back(mesh);
//       triangleCount++;
//     }
//   }

//   data_.vVertexCount_.push_back(vertexCount);
//   data_.vIdxCount_.push_back(triangleCount);
//   std::cout << "\n SoA read " << _filename << ": "
//             << vertexCount << " vertices, "
//             << triangleCount << " triangles"
//             << std::flush;

//   return true;
// }

void Raytracer::pre_read_scene(const std::string &filename)
{
  iVertexCount_ = 0;
  iTriangleCount_ = 0;
  iMeshCount_ = 0;
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
      iMeshCount_++;
    } else {
      continue;
    }
  }
  ifs.close();
  data_.vVertexPos_.reserve(iVertexCount_);
  data_.vVertexIdx_.reserve(iTriangleCount_*3);
  data_.vMeshesPerIdx_.reserve(iTriangleCount_*3);

  data_.vFirstVertex_.reserve(iMeshCount_);
  data_.vVertexCount_.reserve(iMeshCount_);
  data_.vFirstIdx_.reserve(iMeshCount_);
  data_.vIdxCount_.reserve(iMeshCount_);

  std::cout << "\ndone (" << iVertexCount_ << " Vertices)\n";
  std::cout << "\ndone (" << iTriangleCount_ << " Triangles)\n";
  std::cout << "\ndone (" << iMeshCount_ << " Meshes)\n";
}

/**
 * @brief pre reads obj file to enable memory allocation @ pre-read-scene
 *        (to know how many vertices and indices exist)
 */
void Raytracer::pre_read_obj(const char* _filename)
{
  int vertexCount = 0;
  int triangleCount = 0;
  // open obj file
  std::ifstream ifs(_filename);
  if (!ifs) {
    std::cerr << "Can't open " << _filename << "\n";
    return;
  }

  std::string filename(_filename);
  std::string line;
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
    // faces
    if (header == "f") {
      triangleCount++;
      continue;
    }
  }
  iVertexCount_ = iVertexCount_ + vertexCount;
  iTriangleCount_ = iTriangleCount_ + triangleCount;


  std::cout << "\n  read " << _filename << ": "
            << vertexCount << " vertices, "
            << triangleCount << " triangles"
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
  for (const Light &light : vLights_) {

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
