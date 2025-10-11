// Copyright (c) 2025 Hyovin Kwak
// This file contains my original work and depends on the TU Dortmund
// "Computer Graphics" exercise framework, which is not distributed in this repo.
// License: MIT (for this file only)

#include <algorithm>
#include "Raytracer.h"
#include "utils/vec3.h"
#include "utils/Material.h"

/**
 * @brief returns diffuse term
 *
 * @param point the point, of which the color should be determined
 * @param normal normal at the point
 * @param light light source
 */
double Raytracer::diffuse(const vec3 &point, const vec3 &normal, const Light &light) const {
  vec3 l = normalize(light.position - point);
  double dot_nl = dot(normal, l);
  dot_nl = std::max(0.0, dot_nl);
  return dot_nl;
}

/**
 * @brief returns specular reflection term
 *
 * @param point the point, of which the color should be dtermined
 * @param normal normal at the point
 * @param view normalized direction from the 'point' to the viewer's position
 * TODO: parameter exponent could be optional here to control shininess
 */
double Raytracer::reflection(const vec3 &point, const vec3 &normal, const vec3 &view, const Light &light) const {
  if (diffuse(point, normal, light) > 0.0){
    vec3 l = normalize(light.position - point);
    vec3 r = normalize(mirror(l, normal));
    // vec3 r = normalize(2 * normal * dot(normal, l) - l);
    double dot_rv = dot(r, view);
    dot_rv = std::max(0.0, dot_rv);
    return dot_rv;
  }
  return 0.0;
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
  /** TODO
   * Compute reflections by recursive ray tracing:
   * OK - recursion depth check @ tracer() with `max_depth_`
   * OK - check `material.mirror` to check check whether `object` is reflective
   * OK - generate reflected ray, compute its color contribution, and mix it
   *      with the color computed by local Phong lighthing (use `material.mirror` as
   *      weight)
   */
  if (material.mirror > 0.0){
      // generate reflected ray using normal, point, and ray.direction
      vec3 v = mirror(-ray.direction_, normal);
      Ray ray_ = Ray(point, v);
      return material.mirror*trace(ray_, depth+1);
  }
  return {0, 0, 0};
}

/**
 * @brief Computes Phong lighting model (ambient + diffuse + specular) at a
 *        surface point
 * @param point: intersection point
 *        normal: intersection normal
 *        view: ray from intersection point to pixel
 *        material: material of intersection point
 *
 * @return RGB color/radiance of all light sources at point
 */
vec3 Raytracer::lighting(const vec3 &point, const vec3 &normal,
                         const vec3 &view, const Material &material) const {
  vec3 color(0.0, 0.0, 0.0);

  /** TODO
   * Compute the Phong lighting:
   * OK - global ambient lighting
   * OK - diffuse and specular light for each light source
   * OK - add shadow term to implement occlusion due to shadow
   */

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
    double reflection_ = 1.0;
    // TODO: exponent could be optional here to control shininess
    for (int i = 0; i < 80; i++){
      reflection_ *= dot_rv;
    }

    // compute hard shadow term
    Material shadow_material;
    vec3 shadow_point;
    vec3 shadow_normal;
    double shadow_t;
    Ray shadow_ray = Ray(point, normalize(light.position - point));
    bool isShadow = intersect_scene(shadow_ray, shadow_material, shadow_point, shadow_normal, shadow_t);

    color[0] += light.color[0] * !isShadow *(material.diffuse[0] * diffuse_ + material.specular[0] * reflection_);
    color[1] += light.color[1] * !isShadow *(material.diffuse[1] * diffuse_ + material.specular[1] * reflection_);
    color[2] += light.color[2] * !isShadow *(material.diffuse[2] * diffuse_ + material.specular[2] * reflection_);
  }
  return color;
}

//=============================================================================
