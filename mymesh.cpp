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
#include "mydata.h"

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
bool Mesh::intersect_triangle(const Triangle &triangle,
                              const Ray &ray,
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
  if (hasTexture_){
    compute_texture(triangle.iuv0, triangle.iuv1, triangle.iuv2, intersection_diffuse, alpha, beta, gamma);
  }

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


bool Mesh::intersect_triangle_SoA(const vec3& p0,
                                  const vec3& p1,
                                  const vec3& p2,
                                  const vec3& n,
                                  const vec3& vn0,
                                  const vec3& vn1,
                                  const vec3& vn2,
                                  const double& u0,
                                  const double& u1,
                                  const double& u2,
                                  const double& v0,
                                  const double& v1,
                                  const double& v2,
                                  const Ray &ray,
                                  vec3 &intersection_point,
                                  vec3 &intersection_normal,
                                  vec3 &intersection_diffuse,
                                  double &intersection_distance) const
{
  std::cout << "intersectTriangleSoA" << std::endl;

  intersection_diffuse = material_.diffuse;

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
  if (hasTexture_){

    // interpolate with barycentrics
    double u = alpha * u0 + beta * u1 + gamma * u2;
    double v = alpha * v0 + beta * v1 + gamma * v2;

    // (optional) keep inside [0,1]
    u = std::clamp(u, 0.0, 1.0);
    v = std::clamp(v, 0.0, 1.0);

    const unsigned int W = texture_.width();  // u
    const unsigned int H = texture_.height(); // v

    int px = std::round(u * (W - 1));
    int py = std::round((1.0 - v) * (H - 1));

    intersection_diffuse = texture_(px, py);
  }


  if (draw_mode_ == FLAT) {
    // compute flat normal
    intersection_normal = normalize(cross(p1 - p0, p2 - p0));
  } else {
    // TODO: compute normals SoA

    // interpolate vertex normals
    intersection_normal = alpha * vn0 + beta * vn1 + gamma * vn2;
  }
  return true;
}
