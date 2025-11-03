#ifndef MYDATA_H
#define MYDATA_H

#include "utils/vec3.h"
#include <Mesh.h>
#include <vector>

/**
 * @brief SoA for concat data. Note that each vector is..
 *        mbase: based on the number of mesh
 *        vbase: based on the number of vertices
 *        ibase: based on the number of vertex indices
 *        tbase: based on the number of triangles
 *        ibase and tbase could be compatible.
 */
struct Data {

  /// Meshes
  std::vector<Mesh*> meshes_; // vbase!

  /// Data
  /// vertexPos_, vertexNormals_
  std::vector<vec3> vertexPos_;     // vbase
  std::vector<vec3> vertexNormals_; // vbase

  /// Vertex Indices(Triangle)
  std::vector<int> vertexIdx_; // ibase, Indices of vertices

  /// Texture Coordinates
  std::vector<double> textureCoordinatesU_;
  std::vector<double> textureCoordinatesV_;

  /// Texture Indices
  std::vector<int> textureIdx_; // ibase, iuv0, iuv1, iuv2

  /// Normals
  std::vector<vec3> normals_;   // tbase, i0

  //// MeshView
  /// vbase: vertexPos_, vertexNormals_
  std::vector<int> firstVertex_;
  std::vector<int> vertexCount_;

  /// ibase: vertexIdx_
  std::vector<int> firstVertexIdx_;
  std::vector<int> vertexIdxCount_;

  /// textureCoordinatesUV
  std::vector<int> firstTextCoord_;
  std::vector<int> textCoordCount_;

  /// ibase: textureIdx_
  std::vector<int> firstTextIdx_;
  std::vector<int> textIdxCount_;
};


#endif // MYDATA_H
