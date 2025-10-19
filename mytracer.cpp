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

  /** \todo in trace() @ Raytracer.cpp */

  // if (material.mirror > 0.0){

  // }
  // generate reflected ray using normal, point, and ray.direction
  // vec3 v = mirror(-ray.direction_, normal);
  vec3 v = reflect(ray.direction_, normal);
  const double epsilon = 1e-4; // small offset to avoid self-intersection
  Ray ray_ = Ray(point + epsilon * v, v);
  return material.mirror * trace(ray_, depth + 1);
  //return {0, 0, 0};
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

    // compute hard shadow term
    Material shadow_material;
    vec3 shadow_point;
    vec3 shadow_normal;
    double shadow_t;
    vec3 light_direction = normalize(light.position - point);
    double light_distance = norm(light.position - point);
    Ray shadow_ray = Ray(point + epsilon * light_direction, light_direction);
    bool isShadow = intersect_scene(shadow_ray, shadow_material, shadow_point, shadow_normal, shadow_t);
    isShadow = isShadow && shadow_t < light_distance && 0.0 < shadow_t;

    color[0] += light.color[0] * !isShadow * (material.diffuse[0] * diffuse_ + material.specular[0] * reflection_);
    color[1] += light.color[1] * !isShadow * (material.diffuse[1] * diffuse_ + material.specular[1] * reflection_);
    color[2] += light.color[2] * !isShadow * (material.diffuse[2] * diffuse_ + material.specular[2] * reflection_);
  }
  return color;
}

//=============================================================================
