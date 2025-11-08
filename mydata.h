// ============================================================================
// Computer Graphics(Graphische Datenverarbeitung) - TU Dortmund
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


/* struct stMesh { */

/*   bool hasTexture = false; */
/*   unsigned width = 0, height = 0; */
/*   vec4* pixelsTexture = nullptr; */

/*   inline size_t idx(unsigned x, unsigned y) const { */
/*     const unsigned s = width; */
/*     return size_t (y) * s + x; */
/*   } */
/*   inline vec4&       texel(unsigned x, unsigned y)       { return pixelsTexture[idx(x,y)]; } */
/*   inline const vec4& texel(unsigned x, unsigned y) const { return pixelsTexture[idx(x,y)]; } */
/* }; */


/**
 * @brief Structure-of-Arrays (SoA) data layout for GPU-friendly memory access
 *
 * Data is organized by type rather than by object, enabling coalesced memory
 * access on GPUs. Vectors are indexed using different base systems:
 * - vbase: based on vertex count
 * - ibase: based on vertex index count
 * - tbase: based on texture coordinates count
 * Note: ibase and tbase are compatible
 */
struct Data {

  /// ===== total coounts from pre-read-scene =====
  int tMeshCount_ = 0, tVertexCount_ = 0, tVertexIdxCount_ = 0, tTextCoordCount_ = 0, tTextIdxCount_ = 0;
  size_t tTotalTexels_ = 0;

  // ===== Per-vertex data (vbase) =====
  int*  vertexMeshId_ = nullptr;        ///< per-vertex: which mesh this vertex belongs to
  vec4* vertexPos_ = nullptr;           ///< Vertex positions
  vec4* vertexNormals_ = nullptr;       ///< Vertex normals

  // ===== Per-triangle data (ibase/3) =====
  vec4* normals_ = nullptr;           ///< Triangle face normals

  // ===== Index data (ibase) =====
  int* vertexIdx_ = nullptr;          ///< Vertex indices (3 per triangle)
  int* textureIdx_ = nullptr;         ///< Texture coordinate indices (3 per triangle)

  // ===== Texture data (tbase) =====
  double *textureCoordinatesU_ = nullptr; ///< U texture coordinates
  double *textureCoordinatesV_ = nullptr; ///< V texture coordinates

  // Per-mesh metadata (all [meshCount]):
  int* firstVertex_ = nullptr;     int* vertexCount_ = nullptr;
  int* firstVertexIdx_ = nullptr;  int* vertexIdxCount_ = nullptr;
  int* firstTextCoord_ = nullptr;  int* textCoordCount_ = nullptr;
  int* firstTextIdx_ = nullptr;    int* textIdxCount_ = nullptr;

  /// Texture
  // per-mesh texture info (one allocation per mesh for texels)
  int *meshTexWidth_ = nullptr;  // [tMeshCount_]
  int *meshTexHeight_ = nullptr; // [tMeshCount_]
  vec4 *meshTexels_ = nullptr;  // [tMeshCount_] of pointers to [W*H] UM blocks
  size_t* meshTexOffset_= nullptr;   // [tMeshCount_], start index into texels_ for each mesh

};

#endif // MYDATA_H
