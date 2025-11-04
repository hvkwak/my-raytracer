// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#ifndef MYDATA_H
#define MYDATA_H

#include "utils/vec3.h"
#include <Mesh.h>
#include <vector>

/**
 * @brief Structure-of-Arrays (SoA) data layout for GPU-friendly memory access
 *
 * Data is organized by type rather than by object, enabling coalesced memory
 * access on GPUs. Vectors are indexed using different base systems:
 * - mbase: based on mesh count
 * - vbase: based on vertex count
 * - ibase: based on vertex index count
 * - tbase: based on triangle count
 * Note: ibase and tbase are compatible
 */
struct Data {
  // ===== Per-vertex data (vbase) =====
  std::vector<Mesh*> meshes_;           ///< Mesh pointer for each vertex
  std::vector<vec3> vertexPos_;         ///< Vertex positions
  std::vector<vec3> vertexNormals_;     ///< Vertex normals

  // ===== Per-triangle data (tbase) =====
  std::vector<vec3> normals_;           ///< Triangle face normals

  // ===== Index data (ibase) =====
  std::vector<int> vertexIdx_;          ///< Vertex indices (3 per triangle)
  std::vector<int> textureIdx_;         ///< Texture coordinate indices (3 per triangle)

  // ===== Texture data =====
  std::vector<double> textureCoordinatesU_;  ///< U texture coordinates
  std::vector<double> textureCoordinatesV_;  ///< V texture coordinates

  // ===== Mesh view metadata =====
  std::vector<int> firstVertex_;        ///< Starting vertex index per mesh
  std::vector<int> vertexCount_;        ///< Vertex count per mesh

  std::vector<int> firstVertexIdx_;     ///< Starting vertex index offset per mesh
  std::vector<int> vertexIdxCount_;     ///< Vertex index count per mesh

  std::vector<int> firstTextCoord_;     ///< Starting texture coordinate index per mesh
  std::vector<int> textCoordCount_;     ///< Texture coordinate count per mesh

  std::vector<int> firstTextIdx_;       ///< Starting texture index offset per mesh
  std::vector<int> textIdxCount_;       ///< Texture index count per mesh
};


#endif // MYDATA_H
