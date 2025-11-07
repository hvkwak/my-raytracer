// ============================================================================
// Computer Graphics - TU Dortmund
// Implementation by Hyovin Kwak (Instructor: Prof. Dr. Mario Botsch)
//
// This file contains my solutions to the course exercises.
// Note: The original exercise framework/codebase is not published in this repo.
// ============================================================================

#ifndef MYDATA_H
#define MYDATA_H

#include "utils/vec4.h"
#include <Mesh.h>
#include <vector>
#include <cuda_runtime.h>


struct stMesh {
  bool hasTexture = false;
  unsigned width = 0, height = 0;
  vec4* pixels = nullptr;

  inline size_t idx(unsigned x, unsigned y) const {
    const unsigned s = width;
    return size_t (y) * s + x;
  }
  inline vec4&       texel(unsigned x, unsigned y)       { return pixels[idx(x,y)]; }
  inline const vec4& texel(unsigned x, unsigned y) const { return pixels[idx(x,y)]; }
};


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

  /// Variables for pre-read / SoA @ pre_read_scene()
  /* int meshCount_; */
  /* int vertexCount_; */
  /* int vertexIdxCount_; */
  /* int textCoordCount_; */
  /* int textIdxCount_; */

  int tMeshCount = 0, tVertexCount_ = 0, tVertexIdxCount_ = 0, tTextCoordCount_ = 0, tTextIdxCount_ = 0;

  // One stMesh per MESH:
  stMesh* meshes_ = nullptr;          // [meshCount]

  // Per-vertex arrays:
  vec4* vertexPos_ = nullptr;         // [vertexCount]
  vec4* vertexNormals_ = nullptr;     // [vertexCount]
  int*  vertexMeshId_ = nullptr;      // [VertexCount]

  // Per-triangle / index:
  vec4* normals_ = nullptr;           // [vertexIdxCount_/3]
  int*  vertexIdx_ = nullptr;         // [vertexIdxCount_]
  int*  textureIdx_ = nullptr;        // [vertexIdxCount_]

  // Texture coordinates (u/v pairs):
  double* textureCoordinatesU_ = nullptr; // [textCoordCount_]
  double* textureCoordinatesV_ = nullptr; // [textCoordCount_]

  // Per-mesh metadata (all [meshCount]):
  int* firstVertex_ = nullptr;     int* vertexCount_ = nullptr;
  int* firstVertexIdx_ = nullptr;  int* vertexIdxCount_ = nullptr;
  int* firstTextCoord_ = nullptr;  int* textCoordCount_ = nullptr;
  int* firstTextIdx_ = nullptr;    int* textIdxCount_ = nullptr;
};

/* struct Data { */

/*   // ===== Per-vertex data (vbase) ===== */
/*   stMesh* meshes_ = nullptr;           ///< Mesh pointer for each vertex */
/*   vec4* vertexPos_ = nullptr;         ///< Vertex positions */
/*   vec4* vertexNormals_ = nullptr;     ///< Vertex normals */

/*   // ===== Per-triangle data (tbase) ===== */
/*   vec4* normals_ = nullptr;           ///< Triangle face normals */

/*   // ===== Index data (ibase) ===== */
/*   int* vertexIdx_ = nullptr;          ///< Vertex indices (3 per triangle) */
/*   int* textureIdx_ = nullptr;         ///< Texture coordinate indices (3 per triangle) */

/*   // ===== Texture data ===== */
/*   double* textureCoordinatesU_ = nullptr;  ///< U texture coordinates */
/*   double* textureCoordinatesV_ = nullptr;  ///< V texture coordinates */

/*   // ===== Mesh view metadata ===== */
/*   int* firstVertex_ = nullptr;        ///< Starting vertex index per mesh */
/*   int* vertexCount_ = nullptr;        ///< Vertex count per mesh */

/*   int* firstVertexIdx_ = nullptr;     ///< Starting vertex index offset per mesh */
/*   int* vertexIdxCount_ = nullptr;     ///< Vertex index count per mesh */

/*   int* firstTextCoord_ = nullptr;     ///< Starting texture coordinate index per mesh */
/*   int* textCoordCount_ = nullptr;     ///< Texture coordinate count per mesh */

/*   int* firstTextIdx_ = nullptr;       ///< Starting texture index offset per mesh */
/*   int* textIdxCount_ = nullptr;       ///< Texture index count per mesh */
/* }; */
#endif // MYDATA_H
