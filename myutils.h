// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#pragma once

#include "utils/vec3.h"

/**
 * @brief Compute determinant of a 2x2 matrix
 * @param a Element at (0,0)
 * @param b Element at (0,1)
 * @param c Element at (1,0)
 * @param d Element at (1,1)
 * @return Determinant value
 */
double det2D(const double &a, const double & b, const double & c, const double & d);

/**
 * @brief Compute determinant of a 3x3 matrix
 * @param v1 First column vector
 * @param v2 Second column vector
 * @param v3 Third column vector
 * @return Determinant value
 */
double det3D(const vec3 & v1, const vec3 & v2, const vec3 & v3);
