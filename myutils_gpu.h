// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#pragma once

#include "utils/vec4.h"
#include <cuda_runtime.h>

/**
 * @brief Compute determinant of a 2x2 matrix (GPU version)
 * @param a Element at (0,0)
 * @param b Element at (0,1)
 * @param c Element at (1,0)
 * @param d Element at (1,1)
 * @return Determinant value
 */
__inline__ __device__ double det2D_device(double a, double b, double c, double d){
    return a*d - b*c;
};

/**
 * @brief Compute determinant of a 3x3 matrix using vec4 (GPU version)
 * @param v1 First column vector
 * @param v2 Second column vector
 * @param v3 Third column vector
 * @return Determinant value
 */
__inline__ __device__ double det4D_device(const vec4 & v1, const vec4 & v2, const vec4 & v3){
    return v1[0] * det2D_device(v2[1], v3[1], v2[2], v3[2])
         - v2[0] * det2D_device(v1[1], v3[1], v1[2], v3[2])
         + v3[0] * det2D_device(v1[1], v2[1], v1[2], v2[2]);
}
