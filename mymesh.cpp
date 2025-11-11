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
 * @brief Test ray intersection with mesh bounding box using slab method
 *
 * @param ray The ray to test
 * @return true if ray intersects the mesh bounding box
 */
bool Mesh::intersect_bounding_box(const Ray &ray) const {
  // Slab method for ray-AABB intersection
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

  return tmax > 1e-5;  // Ensure intersection is in front of ray
}

/**
 * @brief Compute texture color using barycentric interpolation
 *
 * @param iuv0 Texture coordinate index for first triangle vertex
 * @param iuv1 Texture coordinate index for second triangle vertex
 * @param iuv2 Texture coordinate index for third triangle vertex
 * @param intersection_diffuse Resulting texture color (output)
 * @param alpha Barycentric coordinate for first vertex
 * @param beta Barycentric coordinate for second vertex
 * @param gamma Barycentric coordinate for third vertex
 */
void Mesh::compute_texture(int iuv0,
                           int iuv1,
                           int iuv2,
                           vec4& intersection_diffuse,
                           const double & alpha,
                           const double & beta,
                           const double & gamma) const
{
  // Interpolate texture coordinates with barycentric weights
  double u = alpha * u_coordinates_[iuv0] + beta * u_coordinates_[iuv1] +
             gamma * u_coordinates_[iuv2];
  double v = alpha * v_coordinates_[iuv0] + beta * v_coordinates_[iuv1] +
             gamma * v_coordinates_[iuv2];

  // Clamp to [0,1] range
  u = std::clamp(u, 0.0, 1.0);
  v = std::clamp(v, 0.0, 1.0);

  // Map to texture pixel coordinates
  const unsigned int W = texture_.width();
  const unsigned int H = texture_.height();
  int px = std::round(u * (W - 1));
  int py = std::round((1.0 - v) * (H - 1));

  intersection_diffuse = texture_(px, py);
}

/**
 * @brief Compute mesh normals
 *
 * First computes per-face normals, then accumulates them to vertex normals
 * using angle-weighted averaging for better quality
 */
void Mesh::compute_normals() {
  const double eps = 1e-12;

  // Initialize vertex normals to zero
  for (Vertex &v : vertices_) {
    v.normal = vec4(0, 0, 0);
  }

  // Compute triangle face normals
  for (Triangle &t : triangles_) {
    const vec4 &p0 = vertices_[t.i0].position;
    const vec4 &p1 = vertices_[t.i1].position;
    const vec4 &p2 = vertices_[t.i2].position;
    t.normal = normalize(cross(p1 - p0, p2 - p0));
  }

  // Compute vertex normals with angle-weighted averaging
  for (Triangle &t : triangles_) {
    // Vertex positions
    const vec4 &p0 = vertices_[t.i0].position;
    const vec4 &p1 = vertices_[t.i1].position;
    const vec4 &p2 = vertices_[t.i2].position;

    // vectors
    const vec4 v0 = p1 - p0;
    const vec4 v1 = p2 - p1;
    const vec4 v2 = p0 - p2;

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

  // Normalize vertex normals
  for (Vertex &v : vertices_) {
    v.normal = normalize(v.normal);
  }
}

/**
 * @brief Test ray-triangle intersection (AoS version)
 *
 * @param triangle Triangle to test
 * @param ray Ray to test
 * @param intersection_point Intersection point (output)
 * @param intersection_normal Normal at intersection (output)
 * @param intersection_diffuse Diffuse color at intersection (output)
 * @param intersection_distance Distance to intersection (output)
 * @return true if ray intersects the triangle
 */
bool Mesh::intersect_triangle(const Triangle &triangle,
                              const Ray &ray,
                              vec4 &intersection_point,
                              vec4 &intersection_normal,
                              vec4 &intersection_diffuse,
                              double &intersection_distance) const {
  intersection_diffuse = material_.diffuse;
  const vec4 &p0 = vertices_[triangle.i0].position;
  const vec4 &p1 = vertices_[triangle.i1].position;
  const vec4 &p2 = vertices_[triangle.i2].position;

  // Solve for barycentric coordinates and ray parameter t
  // Equation: ray.origin + t*ray.dir = alpha*p0 + beta*p1 + gamma*p2
  // where gamma = 1 - alpha - beta
  const vec4 column1 = {p0[0] - p2[0], p0[1] - p2[1], p0[2] - p2[2]};
  const vec4 column2 = {p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]};
  const vec4 column3 = {-ray.direction_[0], -ray.direction_[1], -ray.direction_[2]};
  const vec4 column4 = {ray.origin_[0] - p2[0], ray.origin_[1] - p2[1], ray.origin_[2] - p2[2]};
  const double S = det4D(column1, column2, column3);

  // Check for degenerate triangle (determinant near zero)
  if (std::fabs(S) < 1e-10)
    return false;

  const double alpha = det4D(column4, column2, column3) / S;
  const double beta = det4D(column1, column4, column3) / S;
  const double gamma = (1.0 - alpha - beta);
  const double eps_shadow_acne = 1e-5;

  // Check if intersection is in front of ray (avoid shadow acne)
  const double t = det4D(column1, column2, column4) / S;
  if (t <= eps_shadow_acne)
    return false;

  // Check if barycentric coordinates are valid (point is inside triangle)
  bool isInside = (0.0 <= alpha && alpha <= 1.0) &&
                  (0.0 <= beta && beta <= 1.0) &&
                  (0.0 <= gamma && gamma <= 1.0);
  if (!isInside)
    return false;

  // Compute intersection parameters
  intersection_distance = t;
  intersection_point = ray(t);

  // Apply texture if available
  if (hasTexture_){
    compute_texture(triangle.iuv0, triangle.iuv1, triangle.iuv2,
                    intersection_diffuse, alpha, beta, gamma);
  }

  // Compute normal (flat or interpolated)
  if (draw_mode_ == FLAT) {
    intersection_normal = triangle.normal;
  } else {
    intersection_normal = alpha * vertices_[triangle.i0].normal +
                          beta * vertices_[triangle.i1].normal +
                          gamma * vertices_[triangle.i2].normal;
  }
  return true;
}
