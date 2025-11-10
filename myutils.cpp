// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#include "utils/vec3.h"
#include "utils/vec4.h"

// Make this header usable with and without NVCC
#ifndef __CUDACC__
  #define CUDA_HD
#else
  #include <cuda_runtime.h>
  #define CUDA_HD __host__ __device__
#endif

/**
 * @brief returns determinant of a 2x2 matrix
 *
 * @param a element at (0, 0)
 *        b element at (0, 1)
 *        c element at (1, 0)
 *        d element at (1, 1)
 *
 * @return determinant of a 2x2 matrix
 */
CUDA_HD double det2D(double a, double b, double c, double d){
    return a*d - b*c;
}

/**
 * @brief returns determinanat of a 3x3 matrix
 *
 * @param v1: first column of matrix
 *        v2: second column of matrix
 *        v3: third column of matrix
 * @return determinant of a 3x3 matrix
 */
CUDA_HD double det3D(const vec3 & v1, const vec3 & v2, const vec3 & v3){
    return v1[0] * det2D(v2[1], v3[1], v2[2], v3[2])
         - v2[0] * det2D(v1[1], v3[1], v1[2], v3[2])
         + v3[0] * det2D(v1[1], v2[1], v1[2], v2[2]);
}

/**
 * @brief equivalent to det3D()
 *
 * @param v1: first column of matrix
 *        v2: second column of matrix
 *        v3: third column of matrix
 * @return determinant of a 3x3 matrix
 */
CUDA_HD double det4D(const vec4 & v1, const vec4 & v2, const vec4 & v3){
    return v1[0] * det2D(v2[1], v3[1], v2[2], v3[2])
         - v2[0] * det2D(v1[1], v3[1], v1[2], v3[2])
         + v3[0] * det2D(v1[1], v2[1], v1[2], v2[2]);
}
