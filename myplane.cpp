/// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#include "Plane.h"
#include "utils/vec3.h"
#include "utils/Material.h"

//-----------------------------------------------------------------------------
/**
 * @brief returns whether the incoming ray intersect the Plane
 *
 * @param ray: incoming ray
 *        intersection point: point where ray reaches the plane
 *        intersection_normal: normal at the intersection (normal of plane)
 *        intersection_diffuse: diffuse term from the material of Plane
 *        intersection_distance: distance from ray origin to intersection point
 * @return
 */
bool Plane::intersect(const Ray& ray, vec3& intersection_point,
                      vec3& intersection_normal, vec3& intersection_diffuse,
                      double& intersection_distance) const
{
    /** \todo in Plane.cpp */
    intersection_diffuse = material_.diffuse;
    // returns no intersection, if ray and plane are parallel.
    const double epsilon = 1e-9;
    const double cosTheta = dot(normal_, ray.direction_);
    const bool isParallel = std::abs(cosTheta) < epsilon;
    if (isParallel){
        return false;
    }

    // compute intersection data
    double distance = dot(normal_, center_); // distance from origin
    double t = (distance - dot(normal_, ray.origin_))/dot(normal_, ray.direction_);

    // and store it in reference parameters
    // return whether there's an intersection for t>1e-5 to avoid shadow acne.
    if (t > 1e-5){
        intersection_point = ray(t);
        intersection_distance = t;
        intersection_normal = normal_;
        return true;
    }

    return false;
}
//=============================================================================
