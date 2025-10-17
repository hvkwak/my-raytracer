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
// #include <fstream>
// #include <string>
// #include <sstream>
// #include <map>
// #include <cfloat>

bool Mesh::intersect_bounding_box(const Ray& ray) const
{
    /** \todo in Mesh.cpp*/
//     /** \todo
//     * Intersect the ray `_ray` with the axis-aligned bounding box of the mesh.
//     * Note that the minimum and maximum point of the bounding box are stored
//     * in the member variables `bb_min_` and `bb_max_`. Return whether the ray
//     * intersects the bounding box.
//     *
//     * Hints:
//     * - The box intersection is basically a combination of 6 plane/ray intersections.
//     * - The resulting intersection point has to be checked against the lower and upper bounds `bb_min_` and `bb_max_`.
//     * - The intersection tests can be done easier than the existing general ray/plane intersection because the planes are axis aligned.
//     * - One positive intersection is sufficient to return true.
//     * - It helps to visualize a 2D ray/rectangle intersection on a sheet of paper.
//     * - To debug your bounding box code, comment out your triangle intersection test (return true at the beginning) and test
//     *   the cube or toon_faces scene. You should see the black bounding boxes, if everything is correct.
//     * - There are other (faster) ray/box intersection approaches, feel free to implement one of those instead.
//     *
//     * Note:
//     * This function is used in `Mesh::intersect()` to avoid the intersection test
//     * with all triangles of every mesh in the scene. The bounding boxes are computed
//     * in `Mesh::compute_bounding_box()`.
//     */

    vec3 vec_x = (vec3(bb_max_[0] - bb_min_[0], 0.0, 0.0));
    vec3 vec_y = (vec3(0.0, bb_max_[1] - bb_min_[1], 0.0));
    vec3 vec_z = (vec3(0.0, 0.0, bb_max_[2] - bb_min_[2]));

    vec3 center1 = bb_min_ + vec_x/2.0 + vec_y/2.0;
    vec3 center2 = bb_min_ + vec_y/2.0 + vec_z/2.0;
    vec3 center3 = bb_min_ + vec_z/2.0 + vec_x/2.0;
    vec3 center4 = bb_min_ + vec_z + vec_x/2.0 + vec_y/2.0;
    vec3 center5 = bb_min_ + vec_x + vec_y/2.0 + vec_z/2.0;
    vec3 center6 = bb_min_ + vec_y + vec_z/2.0 + vec_x/2.0;

    vec3 normal1 = normalize(cross(vec_x, vec_y));
    vec3 normal2 = normalize(cross(vec_y, vec_z));
    vec3 normal3 = normalize(cross(vec_z, vec_x));
    vec3 normal4 = normalize(cross(vec_y, vec_x));
    vec3 normal5 = normalize(cross(vec_z, vec_y));
    vec3 normal6 = normalize(cross(vec_x, vec_z));

    Plane plane1(center1, normal1);
    Plane plane2(center2, normal2);
    Plane plane3(center3, normal3);
    Plane plane4(center4, normal4);
    Plane plane5(center5, normal5);
    Plane plane6(center6, normal6);
    std::vector<Plane> planes = {plane1, plane2, plane3, plane4, plane5, plane6};

    const double epsilon = 1e-9;
    for (Plane &plane : planes) {

      // check if parallel
      double cosTheta = dot(plane.normal_, ray.direction_);
      bool isParallel = std::abs(cosTheta) < epsilon;
      if (isParallel) {
        continue;
      }

      // compute intersection of ray point on plane
      double distance = dot(plane.normal_, plane.center_); // distance from origin
      double t = distance - (plane.normal_[0] * ray.origin_[0] + plane.normal_[1] * ray.origin_[1] + plane.normal_[2] * ray.origin_[2]);
      t /= (plane.normal_[0] * ray.direction_[0] + plane.normal_[1] * ray.direction_[1] + plane.normal_[2] * ray.direction_[2]);

      if (t > 1e-5) {
        // possible intersectioni poin, check if it is within range
        vec3 point = ray(t);
        bool isX = bb_min_[0] <= point[0] && point[0] <= bb_max_[0];
        bool isY = bb_min_[1] <= point[1] && point[1] <= bb_max_[1];
        bool isZ = bb_min_[2] <= point[2] && point[2] <= bb_max_[2];
        if (isX && isY && isZ) {
          return true;
        }
      }
    }
    return false;

    return true;
}

//-----------------------------------------------------------------------------


/**
 * @brief computes texture coordinates
 *
 * @param
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
  /** \todo in Mesh.cpp
   * Support textured triangles:
   * OK - `hasTexture_` indicates if the mesh is textured.
   * OK - Access the three u and v texture coordinates stored in `u_coordinates`
   *      resp. `v_coordinates` via the triangles iuv indices.
   * OK - Interpolate the uv-coordinates using your barycentric coordinates to
   *      get the intersection point's uv.
   * OK - Convert the relative uv coordinates (from 0 to 1) to absolute pixel
   *   coordinates (from 0 to width/height - 1 of `texture_`)
   * OK - Store the resulting texture color in `intersection_diffuse`
   * - You will notice that there will be shadows on the sky mesh in the
   *   pokemon scene. Use `material.shadowable` in the `lighting(...)` function
   *   to prevent it from being shadowed.
   * (`material.shadowable` is already set to false for the sky mesh and true
   * for all other meshes, so you don't have to set it by yourself)
   */

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

    // Angle-weight denominators: ||u||*||v|| + dot(u,v)
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
bool Mesh::intersect_triangle(const Triangle& triangle, const Ray& ray,
                              vec3& intersection_point,
                              vec3& intersection_normal,
                              vec3& intersection_diffuse,
                              double& intersection_distance) const
{
  return true;
    /** \todo in Mesh.cpp*/

    intersection_diffuse = material_.diffuse;
    const vec3& p0 = vertices_[triangle.i0].position;
    const vec3& p1 = vertices_[triangle.i1].position;
    const vec3& p2 = vertices_[triangle.i2].position;

    // rearranged `ray.origin + t*ray.dir = a*p0 + b*p1 + (1-a-b)*p2`
    // [ p0.x - p2.x   p1.x - p2.x   -d.x ] [a]   [ray.origin.x]
    // [ p0.y - p2.y   p1.y - p2.y   -d.y ]*[b] = [ray.origin.y]
    // [ p0.z - p2.z   p1.z - p2.z   -d.z ] [t]   [ray.origin.z]
    const vec3 column1 = {p0[0] - p2[0], p0[1] - p2[1], p0[2] - p2[2]};
    const vec3 column2 = {p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]};
    const vec3 column3 = {-ray.direction_[0], -ray.direction_[1], -ray.direction_[2]};
    const vec3 column4 = {ray.origin_[0]-p2[0], ray.origin_[1]-p2[1], ray.origin_[2]-p2[2]};
    const double S = det3D(column1, column2, column3);
    const double alpha = det3D(column4, column2, column3)/S;
    const double beta = det3D(column1, column4, column3)/S;
    const double gamma = (1.0 - alpha - beta);
    const double eps_shadow_acne = 1e-5; // TODO: more eps values for barycentric coordiantes?

    // check if t is correct: positive && beyond shadow acne
    const double t = det3D(column1, column2, column4)/S;
    if (t <= eps_shadow_acne) return false;

    // check if it's inside
    bool isInside = true; // shadow acne guard
    isInside = isInside && 0.0 <= alpha && alpha <= 1.0;
    isInside = isInside && 0.0 <= beta && beta <= 1.0;
    isInside = isInside && 0.0 <= gamma && gamma <= 1.0;
    if (!isInside) return false;

    // save intersection parameters
    intersection_distance = t;
    intersection_point = ray(t);

    if (hasTexture_) compute_texture(triangle.iuv0, triangle.iuv1, triangle.iuv2, intersection_diffuse, alpha, beta, gamma);

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
