// ============================================================================
// Solutions/Implementations by Hyovin Kwak to the course
// Computer Graphics @ TU Dortmund (Instructor: Prof. Dr. Mario Botsch)
//
// Note: The original exercise codebase is not included in this repo.
// ============================================================================

#include "Plane.h"
#include "utils/vec4.h"
#include "utils/Material.h"

/**
 * @brief Test ray-plane intersection
 *
 * @param ray Ray to test
 * @param intersection_point Point where ray intersects the plane (output)
 * @param intersection_normal Normal at intersection (output)
 * @param intersection_diffuse Diffuse color at intersection (output)
 * @param intersection_distance Distance from ray origin to intersection (output)
 * @return true if ray intersects the plane
 */
bool Plane::intersect(const Ray& ray, vec4& intersection_point,
                      vec4& intersection_normal, vec4& intersection_diffuse,
                      double& intersection_distance) const
{
    intersection_diffuse = material_.diffuse;

    // Check if ray is parallel to plane
    const double epsilon = 1e-9;
    const double cosTheta = dot(normal_, ray.direction_);
    const bool isParallel = std::abs(cosTheta) < epsilon;
    if (isParallel){
        return false;
    }

    // Compute intersection distance
    double distance = dot(normal_, center_);
    double t = (distance - dot(normal_, ray.origin_)) / dot(normal_, ray.direction_);

    // Check if intersection is in front of ray (avoid shadow acne)
    if (t > 1e-5){
        intersection_point = ray(t);
        intersection_distance = t;
        intersection_normal = normal_;
        return true;
    }

    return false;
}
