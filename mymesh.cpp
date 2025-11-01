/// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#include "Mesh.h"
#include "Plane.h"
#include "myutils.h"
#include <algorithm>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <cfloat>
#include "utils/Data.h"

Mesh::Mesh(Data& Data_,
           Draw_mode _draw_mode,
           const std::string& _filename)
{
    // set draw mode
    draw_mode_ = _draw_mode;

    hasTexture_ = false;

    // load mesh from file
    read_obj(Data_, _filename.c_str());
}

/**
 * @brief read mesh verticies and tri faces in concat vectors
 *
 * @param
 * @return
 */
bool Mesh::read_obj(Data & Data_, const char *_filename)
{
  int vbase = Data_.vVertexPos_.size();
  int ibase = Data_.vVertexIndex_.size();
  Data_.vFirstVertex_.push_back(vbase);
  Data_.vFirstIndex_.push_back(ibase);

  int vertexCount = 0;
  int triangleCount = 0;

  // open obj file
  std::ifstream ifs(_filename);
  if (!ifs) {
    std::cerr << "Can't open " << _filename << "\n";
    return false;
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
      vec3 v;
      lineData >> v[0] >> v[1] >> v[2];
      Data_.vVertexPos_.push_back(v);
      vertexCount++;
      continue;
    }

    // uv-coordinates
    if (header == "vt") {
      hasUV = true;

      double u, v;

      lineData >> u >> v;

      if (u > 1.0 || u < 0.0)
        u -= floor(u);
      if (v > 1.0 || v < -0.0)
        v -= floor(v);

      u_coordinates_.push_back(u);
      v_coordinates_.push_back(v);
      continue;
    }

    if (header == "vn") {
      hasNormals = true;
      continue;
    }

    // material file
    if (header == "mtllib") {
      std::stringstream mtl;
      mtl << filename.substr(0, filename.find_last_of("/") + 1)
          << lineData.str();

      if (!read_mtl(mtl.str(), textures)) {
        std::cerr << "Cannot read mtl file " << mtl.str() << std::endl;
      }

      if (textures.size() > 0)
        hasTexture_ = true;

      continue;
    }

    // start of new material
    if (header == "usemtl") {
      counter++;
      continue;
    }

    // faces
    if (header == "f") {
      Triangle t;

      int uv[3];

      enum { NORMALS, UV, BOTH, NONE } nuv_status;
      if (hasUV)
        nuv_status = hasNormals ? BOTH : UV;
      else
        nuv_status = hasNormals ? NORMALS : NONE;

      // dummy variables for / and normal indices
      int d1;
      char d2;

      // read in face indices and uv indices, skip normal indices
      switch (nuv_status) {
      case BOTH:
        // format: index0/texture0/normal0 index1/texture1/normal1
        // index2/texture2/normal2
        lineData >> t.i0 >> d2 >> uv[0] >> d2 >> d1;
        lineData >> t.i1 >> d2 >> uv[1] >> d2 >> d1;
        lineData >> t.i2 >> d2 >> uv[2] >> d2 >> d1;

        uv[0]--;
        uv[1]--;
        uv[2]--;
        t.iuv0 = uv[0];
        t.iuv1 = uv[1];
        t.iuv2 = uv[2];
        break;
      case UV:
        // format: index0/texture0 index1/texture1 index2/texture2
        lineData >> t.i0 >> d2 >> uv[0];
        lineData >> t.i1 >> d2 >> uv[1];
        lineData >> t.i2 >> d2 >> uv[2];

        uv[0]--;
        uv[1]--;
        uv[2]--;
        t.iuv0 = uv[0];
        t.iuv1 = uv[1];
        t.iuv2 = uv[2];
      case NORMALS:
        // format: index0//normal0 index1//normal1 index2//normal2
        lineData >> t.i0 >> d2 >> d2 >> d1;
        lineData >> t.i1 >> d2 >> d2 >> d1;
        lineData >> t.i2 >> d2 >> d2 >> d1;
      case NONE:
        // format: index0 index1 index2
        lineData >> t.i0 >> t.i1 >> t.i2;
      }

      // decrement because obj indices start by 1
      t.i0--;
      t.i1--;
      t.i2--;

      // convert uv coordinates s.th. we can use just one big combined tex
      // instead of multiple ones
      for (int i = 0; i < 3 && hasUV; i++) {
        if (!uvDone[uv[i]]) {
          int combinedW = 0;
          for (int i = 0; i < counter; i++) {
            combinedW += textures[i].width();
          }
          u_coordinates_[uv[i]] =
              (u_coordinates_[uv[i]] * textures[counter].width() + combinedW) /
              static_cast<double>(texture_.width());
          v_coordinates_[uv[i]] =
              (v_coordinates_[uv[i]] * textures[counter].height()) /
              static_cast<double>(texture_.height());
          uvDone[uv[i]] = true;
        }
      }
      Data_.vVertexIndex_.push_back(vbase + t.i0);
      Data_.vVertexIndex_.push_back(vbase + t.i1);
      Data_.vVertexIndex_.push_back(vbase + t.i2);
      triangleCount++;
    }
  }
  Data_.vVertexCount_.push_back(vertexCount);
  Data_.vIndexCount_.push_back(triangle_count);
  std::cout << "\n  read " << _filename << ": "
            << vertexCount << " vertices, "
            << triangleCount << " triangles"
            << std::flush;

  return true;
}

/**
 * @brief returns if ray intersects a mesh bounding box
 *        based on Slab method
 *
 * @param ray - ray, of which intersection is tested
 * @return true, if ray intersects a mesh bounding box
 */
bool Mesh::intersect_bounding_box(const Ray &ray) const {

  /** \todo in Mesh.cpp*/
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

//-----------------------------------------------------------------------------


/**
 * @brief computes texture coordinates
 *
 * @param iuv0 - index for texture of the first corner of triangle
 *        iuv1 - index for texture of the second corner of triangle
 *        iuv2 - index for texture of the last corner of triangle
 *        intersection_defuse - diffuse lighting term
 *        alpha, beta, gamma: barycentric coordinates
 *
 * @return
 */
void Mesh::compute_texture(int iuv0,
                           int iuv1,
                           int iuv2,
                           vec3& intersection_diffuse,
                           const double & alpha,
                           const double & beta,
                           const double & gamma) const
{
  /** \todo in Mesh.cpp*/

  // interpolate with barycentrics
  double u = alpha * u_coordinates_[iuv0] + beta * u_coordinates_[iuv1] +
             gamma * u_coordinates_[iuv2];
  double v = alpha * v_coordinates_[iuv0] + beta * v_coordinates_[iuv1] +
             gamma * v_coordinates_[iuv2];

  // (optional) keep inside [0,1]
  u = std::clamp(u, 0.0, 1.0);
  v = std::clamp(v, 0.0, 1.0);

  const unsigned int W = texture_.width();  // u
  const unsigned int H = texture_.height(); // v

  int px = std::round(u * (W - 1));
  int py = std::round((1.0 - v) * (H - 1));

  intersection_diffuse = texture_(px, py);
}

/**
 * @brief computes normals of triangular mesh
 *        1. computes triangle face normals
 *        2. computes vertex normals
 *
 * @param
 * @return
 */
void Mesh::compute_normals() {

  /** \todo in Mesh.cpp*/

  const double eps = 1e-12;

  // initialize vertex normals to zero
  for (Vertex &v : vertices_) {
    v.normal = vec3(0, 0, 0);
  }

  // compute triangle normals
  for (Triangle &t : triangles_) {
    const vec3 &p0 = vertices_[t.i0].position;
    const vec3 &p1 = vertices_[t.i1].position;
    const vec3 &p2 = vertices_[t.i2].position;
    t.normal = normalize(cross(p1 - p0, p2 - p0));
  }

  // compute vertex normals
  // per triangle update all three vertices normals
  for (Triangle &t : triangles_) {

    // positions
    const vec3 &p0 = vertices_[t.i0].position;
    const vec3 &p1 = vertices_[t.i1].position;
    const vec3 &p2 = vertices_[t.i2].position;

    // vectors
    const vec3 v0 = p1 - p0;
    const vec3 v1 = p2 - p1;
    const vec3 v2 = p0 - p2;

    // vector lengths
    const double length0 = norm(v0);
    const double length1 = norm(v1);
    const double length2 = norm(v2);

    // dot products of corners
    const double d0 = dot(v0, -v2);
    const double d1 = dot(v1, -v0);
    const double d2 = dot(v2, -v1);

    // Angle-weight denominators: ||u|| * ||v|| + dot(u,v)
    // This equals 2*||u||*||v||*cos^2(theta/2), stays stable near 0.
    const double w0_den = length0 * length2 + d0;
    const double w1_den = length1 * length0 + d1;
    const double w2_den = length2 * length1 + d2;

    // Accumulate: angle-weighted normal contributions
    if (std::abs(w0_den) > eps) {
      vertices_[t.i0].normal += t.normal / w0_den;
    }
    if (std::abs(w1_den) > eps) {
      vertices_[t.i1].normal += t.normal / w1_den;
    }
    if (std::abs(w2_den) > eps) {
      vertices_[t.i2].normal += t.normal / w2_den;
    }
  }

  // Final normalize exactly once per vertex
  for (Vertex &v : vertices_) {
    v.normal = normalize(v.normal);
  }
}

//-----------------------------------------------------------------------------
/**
 * @brief returns true, if ray intersects the given triangle
 *
 * @param triangle - the triangle, of which intersection will be tested
 *        ray      - ray
 *        intersection_point - ray-triangle intersection point
 *        intersection_normal - normal at theh intersection
 *        intersection_diffuse - diffuse term of the intersection
 * @return true, if ray intersects the given triangle.
 */
bool Mesh::intersect_triangle(const Triangle &triangle, const Ray &ray,
                              vec3 &intersection_point,
                              vec3 &intersection_normal,
                              vec3 &intersection_diffuse,
                              double &intersection_distance) const {
  /** \todo in Mesh.cpp*/

  intersection_diffuse = material_.diffuse;
  const vec3 &p0 = vertices_[triangle.i0].position;
  const vec3 &p1 = vertices_[triangle.i1].position;
  const vec3 &p2 = vertices_[triangle.i2].position;

  // rearranged `ray.origin + t*ray.dir = a*p0 + b*p1 + (1-a-b)*p2`
  // [ p0.x - p2.x   p1.x - p2.x   -d.x ] [a]   [ray.origin.x]
  // [ p0.y - p2.y   p1.y - p2.y   -d.y ]*[b] = [ray.origin.y]
  // [ p0.z - p2.z   p1.z - p2.z   -d.z ] [t]   [ray.origin.z]
  const vec3 column1 = {p0[0] - p2[0], p0[1] - p2[1], p0[2] - p2[2]};
  const vec3 column2 = {p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]};
  const vec3 column3 = {-ray.direction_[0], -ray.direction_[1], -ray.direction_[2]};
  const vec3 column4 = {ray.origin_[0] - p2[0], ray.origin_[1] - p2[1], ray.origin_[2] - p2[2]};
  const double S = det3D(column1, column2, column3);
  const double alpha = det3D(column4, column2, column3) / S;
  const double beta = det3D(column1, column4, column3) / S;
  const double gamma = (1.0 - alpha - beta);
  const double eps_shadow_acne = 1e-5;
  // TODO: more eps values for barycentric coordiantes?

  // check if t is correct: positive && beyond shadow acne
  const double t = det3D(column1, column2, column4) / S;
  if (t <= eps_shadow_acne)
    return false;

  // check if it's inside
  bool isInside = true; // shadow acne guard
  isInside = isInside && 0.0 <= alpha && alpha <= 1.0;
  isInside = isInside && 0.0 <= beta && beta <= 1.0;
  isInside = isInside && 0.0 <= gamma && gamma <= 1.0;
  if (!isInside)
    return false;

  // save intersection parameters
  intersection_distance = t;
  intersection_point = ray(t);

  // get Texture if it's there.
  if (hasTexture_)
    compute_texture(triangle.iuv0, triangle.iuv1, triangle.iuv2,
                    intersection_diffuse, alpha, beta, gamma);

  if (draw_mode_ == FLAT) {
    // flat normal
    intersection_normal = triangle.normal;
  } else {
    // interpolate vertex normals
    intersection_normal = alpha * vertices_[triangle.i0].normal +
                          beta * vertices_[triangle.i1].normal +
                          gamma * vertices_[triangle.i2].normal;
  }
  return true;
}

//=============================================================================
