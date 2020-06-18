#include "mesh_tools.hpp"

#include "math_helper.hpp"

#include <execution>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>

namespace loader
{
std::pair<float3, float3>
calculateTangentBitangent(const std::array<float3, 3> &pos,
                          const std::array<float2, 3> &tex)
{
  float3 posEdge1{ };
  float3 posEdge2{ };
  float3 tEdge1{ };
  float3 tEdge2{ };

  posEdge1.x = pos[1].x - pos[0].x;
  posEdge1.y = pos[1].y - pos[0].y;
  posEdge1.z = pos[1].z - pos[0].z;

  posEdge2.x = pos[2].x - pos[0].x;
  posEdge2.y = pos[2].y - pos[0].y;
  posEdge2.z = pos[2].z - pos[0].z;

  tEdge1.x = tex[1].x - tex[0].x;
  tEdge1.y = tex[1].y - tex[0].y;

  tEdge2.x = tex[2].x - tex[0].x;
  tEdge2.y = tex[2].y - tex[0].y;

  const auto den = 1.F / (tEdge1.x * tEdge2.y - tEdge2.x * tEdge1.y);

  float3 tan{ };
  tan.x = (tEdge2.y * posEdge1.x - tEdge1.y * posEdge2.x) * den;
  tan.y = (tEdge2.y * posEdge1.y - tEdge1.y * posEdge2.y) * den;
  tan.z = (tEdge2.y * posEdge1.z - tEdge1.y * posEdge2.z) * den;

  float3 bitan{ };
  bitan.x = (tEdge1.x * posEdge2.x - tEdge2.y * posEdge1.x) * den;
  bitan.y = (tEdge1.x * posEdge2.y - tEdge2.y * posEdge1.y) * den;
  bitan.z = (tEdge1.x * posEdge2.z - tEdge2.y * posEdge1.z) * den;

  auto normalize = [](auto &vec) {
    const float len =
        sqrtf((vec.x * vec.x) + (vec.y * vec.y) + (vec.z * vec.z));

    vec.x = vec.x / len;
    vec.y = vec.y / len;
    vec.z = vec.z / len;
  };

  normalize(tan);
  normalize(bitan);

  return { tan, bitan };
}

bool calcBitan(const float3 &norm, const float4 &tan, float3 &bitan)
{
  bitan = cross(norm, { tan.x, tan.y, tan.z });

  const auto len =
      ((bitan.x * bitan.x) + (bitan.y * bitan.y) + (bitan.z * bitan.z));

  bitan.x /= len * tan.w;
  bitan.y /= len * tan.w;
  bitan.z /= len * tan.w;

  return true;
}
bool calcBitan(const float3 &norm, const float3 &tan, float3 &bitan)
{
  bitan = cross(norm, tan);

  const auto len =
      ((bitan.x * bitan.x) + (bitan.y * bitan.y) + (bitan.z * bitan.z));

  bitan.x /= len;
  bitan.y /= len;
  bitan.z /= len;

  return true;
}
std::unordered_map<Vertex, std::vector<Vertex>, VertexHash> getVertexAdjacency(
    const std::vector<float3> &positions, const std::vector<float3> &normals,
    const std::vector<float2> &uvs, const std::vector<std::uint32_t> &indices)
{
  std::unordered_map<Vertex, std::vector<Vertex>, VertexHash> ret;
  const auto hasNormals = !normals.empty( );
  const auto hasUvs = !uvs.empty( );

  const auto numIndices = indices.size( );
  for (size_t i = 0; i < numIndices; i += 3)
  {
    const auto v1 = indices[i];
    const auto v2 = indices[i + 1];
    const auto v3 = indices[i + 2];

    const float3 empty{ };
    const float2 empty1{ };

    const Vertex vertex1{ .position{ positions[v1] },
                          .normal{ hasNormals ? normals[v1] : empty },
                          .texCord{ hasUvs ? uvs[v1] : empty1 } };

    const Vertex vertex2{ .position{ positions[v2] },
                          .normal{ hasNormals ? normals[v2] : empty },
                          .texCord{ hasUvs ? uvs[v2] : empty1 } };

    const Vertex vertex3{ .position{ positions[v3] },
                          .normal{ hasNormals ? normals[v3] : empty },
                          .texCord{ hasUvs ? uvs[v3] : empty1 } };
    ret[vertex1].push_back(vertex2);
    ret[vertex1].push_back(vertex3);
    ret[vertex2].push_back(vertex1);
    ret[vertex2].push_back(vertex3);
    ret[vertex3].push_back(vertex1);
    ret[vertex3].push_back(vertex2);
  }

  return ret;
}
std::unordered_map<Edge, std::vector<Edge>, EdgeHash> getEdgeAdjacency(
    const std::vector<float3> &positions, const std::vector<float3> &normals,
    const std::vector<float2> &uvs, const std::vector<std::uint32_t> &indices)
{
  std::unordered_map<Edge, std::vector<Edge>, EdgeHash> ret;

  const auto hasNormals = !normals.empty( );
  const auto hasUvs = !uvs.empty( );

  const auto &numIndices = indices.size( );
  for (size_t i = 0; i < numIndices; i += 3)
  {
    const auto &v1 = indices[i];
    const auto &v2 = indices[i + 1];
    const auto &v3 = indices[i + 2];

    const float3 empty{ };
    const float2 empty1{ };

    const Vertex vertex1{ .position{ positions[v1] },
                          .normal{ hasNormals ? normals[v1] : empty },
                          .texCord{ hasUvs ? uvs[v1] : empty1 } };

    const Vertex vertex2{ .position{ positions[v2] },
                          .normal{ hasNormals ? normals[v2] : empty },
                          .texCord{ hasUvs ? uvs[v2] : empty1 } };

    const Vertex vertex3{ .position{ positions[v3] },
                          .normal{ hasNormals ? normals[v3] : empty },
                          .texCord{ hasUvs ? uvs[v3] : empty1 } };

    const Edge e1 = { vertex1, vertex2 };
    const Edge e2 = { vertex1, vertex3 };
    const Edge e3 = { vertex3, vertex1 };

    ret[e1].push_back(e2);
    ret[e1].push_back(e3);
    ret[e2].push_back(e1);
    ret[e2].push_back(e3);
    ret[e3].push_back(e1);
    ret[e3].push_back(e2);
  }
  return ret;
}

STriangleDistanceResult triangleUnsignedDistance(const float3 &from,
                                                 const float3 &v0,
                                                 const float3 &v1,
                                                 const float3 &v2)
{
  const auto diff = v0 - from;
  const auto edge0 = v1 - v0;
  const auto edge1 = v2 - v0;
  const auto a00 = dot(edge0, edge0);
  const auto a01 = dot(edge0, edge1);
  const auto a11 = dot(edge1, edge1);
  const auto b0 = dot(diff, edge0);
  const auto b1 = dot(diff, edge1);
  const auto det = fabsf(a00 * a11 - a01 * a01);

  auto s = a01 * b1 - a11 * b0;
  auto t = a01 * b0 - a00 * b1;

  EDistanceType hit_type;
  if (s + t <= det)
  {
    if (s < 0)
    {
      if (t < 0)// region 4
      {
        if (b0 < 0)
        {
          t = 0;
          if (-b0 >= a00)
          {
            // VN1
            hit_type = EDistanceType::VERT2;
            s = 1;
          }
          else
          {
            // EN0
            hit_type = EDistanceType::EDGE1;
            s = -b0 / a00;
          }
        }
        else
        {
          s = 0;

          if (b1 >= 0)
          {
            // VN0
            hit_type = EDistanceType::VERT1;
            t = 0;
          }
          else if (-b1 >= a11)
          {
            // VN2
            hit_type = EDistanceType::VERT3;
            t = 1;
          }
          else
          {
            // EN2
            hit_type = EDistanceType::EDGE3;
            t = -b1 / a11;
          }
        }
      }
      else// region 3
      {
        s = 0;

        if (b1 >= 0)
        {
          // VN0
          hit_type = EDistanceType::VERT1;
          t = 0;
        }
        else if (-b1 >= a11)
        {
          // VN2
          hit_type = EDistanceType::VERT3;
          t = 1;
        }
        else
        {
          // EN2
          hit_type = EDistanceType::EDGE3;
          t = -b1 / a11;
        }
      }
    }
    else if (t < 0)// region 5
    {
      t = 0;

      if (b0 >= 0)
      {
        // VN0
        hit_type = EDistanceType::VERT1;
        s = 0;
      }
      else if (-b0 >= a00)
      {
        // VN1
        hit_type = EDistanceType::VERT2;
        s = 1;
      }
      else
      {
        // EN0
        hit_type = EDistanceType::EDGE1;
        s = -b0 / a00;
      }
    }
    else// region 0
    {
      // FN
      hit_type = EDistanceType::FACE;
      // minimum at interior point
      auto invDet = (1) / det;
      s *= invDet;
      t *= invDet;
    }
  }
  else
  {
    float tmp0{ };
    float tmp1{ };
    float numer{ };
    float denom{ };

    if (s < 0)// region 2
    {
      tmp0 = a01 + b0;
      tmp1 = a11 + b1;

      if (tmp1 > tmp0)
      {
        numer = tmp1 - tmp0;
        denom = a00 - (2) * a01 + a11;

        if (numer >= denom)
        {
          // VN1
          hit_type = EDistanceType::VERT2;
          s = 1;
          t = 0;
        }
        else
        {
          // EN1
          hit_type = EDistanceType::EDGE2;
          s = numer / denom;
          t = 1 - s;
        }
      }
      else
      {
        s = 0;

        if (tmp1 <= 0)
        {
          // VN2
          hit_type = EDistanceType::VERT3;
          t = 1;
        }
        else if (b1 >= 0)
        {
          // VN0
          hit_type = EDistanceType::VERT1;
          t = 0;
        }
        else
        {
          // EN2
          hit_type = EDistanceType::EDGE3;
          t = -b1 / a11;
        }
      }
    }
    else if (t < 0)// region 6
    {
      tmp0 = a01 + b1;
      tmp1 = a00 + b0;

      if (tmp1 > tmp0)
      {
        numer = tmp1 - tmp0;
        denom = a00 - 2 * a01 + a11;

        if (numer >= denom)
        {
          // VN2
          hit_type = EDistanceType::VERT3;
          t = 1;
          s = 0;
        }
        else
        {
          // EN1
          hit_type = EDistanceType::EDGE2;
          t = numer / denom;
          s = 1 - t;
        }
      }
      else
      {
        t = 0;

        if (tmp1 <= 0)
        {
          // VN1
          hit_type = EDistanceType::VERT2;
          s = 1;
        }
        else if (b0 >= 0)
        {
          // VN0
          hit_type = EDistanceType::VERT1;
          s = 0;
        }
        else
        {
          // EN0
          hit_type = EDistanceType::EDGE1;
          s = -b0 / a00;
        }
      }
    }
    else// region 1
    {
      numer = a11 + b1 - a01 - b0;

      if (numer <= 0)
      {
        // VN2
        hit_type = EDistanceType::VERT3;
        s = 0;
        t = 1;
      }
      else
      {
        denom = a00 - (2) * a01 + a11;

        if (numer >= denom)
        {
          // VN1
          hit_type = EDistanceType::VERT2;
          s = 1;
          t = 0;
        }
        else
        {
          // EN1
          hit_type = EDistanceType::EDGE2;
          s = numer / denom;
          t = 1 - s;
        }
      }
    }
  }

  const auto pnearest = v0 + edge0 * s + edge1 * t;
  STriangleDistanceResult res{ };
  res.hit_type = hit_type;
  res.hit_point = pnearest;
  res.distance = (pnearest.x - from.x) * (pnearest.x - from.x) +
                 (pnearest.y - from.y) * (pnearest.y - from.y) +
                 (pnearest.z - from.z) * (pnearest.z - from.z);
  return res;
}
std::pair<VertexAdjacencyMatrix<std::uint32_t>, EdgeAdjacencyMatrix>
getEdgeAdjacencyIndexed(std::vector<std::uint32_t> &indices)
{
  auto vertexTable = getVertexAdjacencyIndex(indices);

  EdgeAdjacencyMatrix ret;

  const auto &numIndex = indices.size( );
  for (std::uint32_t index = 0; index < numIndex; index += 3)
  {
    const auto i1 = indices[index];
    const auto i2 = indices[index + 1];
    const auto i3 = indices[index + 2];

    auto parseVertex = [&](const auto edge, const auto vertex) {
      const auto arr = vertexTable[indices[vertex]];
      for (const auto &item : arr)
      {
        const auto cmpVert = item;
        const EdgeIndexed place{ std::min(vertex, cmpVert),
                                 std::max(vertex, cmpVert) };
        if (edge != place)
        {
          ret[edge].insert(place);
        }
      }
    };
    auto parseVertices = [&](const auto edge, const auto index1,
                             const auto index2) {
      parseVertex(edge, index1);
      parseVertex(edge, index2);
    };

    const EdgeIndexed e1{ std::min(i1, i2), std::max(i2, i1) };
    const EdgeIndexed e2{ std::min(i2, i3), std::max(i3, i2) };
    const EdgeIndexed e3{ std::min(i3, i1), std::max(i1, i3) };

    parseVertices(e1, index, index + 1);
    parseVertices(e2, index + 1, index + 2);
    parseVertices(e3, index + 2, index + 1);
  }

  return std::pair{ std::move(vertexTable), std::move(ret) };
}
float3 calculateFaceNormal(const float3 &v0, const float3 &v1, const float3 &v2)
{
  const auto e0 = v0 - v1;
  const auto e1 = v0 - v2;

  return normalize(cross(e0, e1));
}

std::vector<float3>
generateMeshFaceNormals(const std::vector<std::uint32_t> &indices,
                        const std::vector<float3> &positions)
{
  const auto &numIndices = indices.size( );
  std::vector<float3> normals(numIndices / 3);
  // Iterate over all faces
  for (std::size_t index = 0; index < numIndices; index += 3)
  {
    const auto &v0 = positions[indices[index]];
    const auto &v1 = positions[indices[index + 1]];
    const auto &v2 = positions[indices[index + 2]];

    normals[index / 3] = calculateFaceNormal(v0, v1, v2);
  }
  return normals;
}

float3 calculatePseudoNormal(
    const STriangleDistanceResult &distanceRes, const std::uint32_t index,
    const std::vector<float3> &positions, const std::vector<uint32_t> &indices,
    const std::vector<float3> &faceNormals,
    const std::pair<VertexAdjacencyMatrix<std::uint32_t>, EdgeAdjacencyMatrix>
        &adjacency)
{
  const auto &ind = indices[index];
  const auto &ind1 = indices[index + 1];
  const auto &ind2 = indices[index + 2];
  switch (distanceRes.hit_type)
  {
  case EDistanceType::VERT1:
    return getPseudoNormalVertex(ind, adjacency.first, positions, indices,
                                 faceNormals);
  case EDistanceType::VERT2:
    return getPseudoNormalVertex(ind1, adjacency.first, positions, indices,
                                 faceNormals);
  case EDistanceType::VERT3:
    return getPseudoNormalVertex(ind2, adjacency.first, positions, indices,
                                 faceNormals);
  case EDistanceType::EDGE1:
  case EDistanceType::EDGE2:
  case EDistanceType::EDGE3:
    return getPseudoNormalEdge(distanceRes, index, adjacency.second, indices,
                               faceNormals);
  case EDistanceType::FACE:
    return faceNormals[index / 3];
  default:
    throw std::runtime_error{ "ERROR: Unknown distance type!" };
  }
}
float3 getPseudoNormalVertex(
    const std::uint32_t v_nearest,
    const VertexAdjacencyMatrix<std::uint32_t> &vertexAdjacencyMatrix,
    const std::vector<float3> &positions,
    const std::vector<std::uint32_t> &indices,
    const std::vector<float3> &faceNormals)
{
  float3 p_normal{ };
  // auto v_nearest = mesh->m_Vertices[tri];

  // This returns an index to an index buffer position.
  for (const auto &adj_tri : vertexAdjacencyMatrix.at(v_nearest))
  {

    // This will adjust index for the first vertex of a face.
    const auto offset = adj_tri % 3;
    const auto firstVertex = adj_tri - offset;

    const auto vert0{ indices[firstVertex] };
    const auto vert1{ indices[firstVertex + 1] };
    const auto vert2{ indices[firstVertex + 2] };

    // This implementation looks less bs than the previous one
    // And has better performance due to less branching.
    const auto v0 = positions[vert0];
    const auto v1 = positions[vert1];
    const auto v2 = positions[vert2];
    const auto e0 = v0 - v1;
    const auto e1 = v0 - v2;

    auto length = [](const float3 &vert) { return sqrtf(dot(vert, vert)); };
    // Do I need this?
    auto incidentAngle = acosf(dot(e0, e1) / (length(e0) * length(e1)));
    p_normal += faceNormals[firstVertex / 3] * incidentAngle;
  }

  return normalize(p_normal);
}

float3 getPseudoNormalEdge(const STriangleDistanceResult &res,
                           const std::uint32_t tri,
                           const EdgeAdjacencyMatrix &adjacencyMatrix,
                           const std::vector<std::uint32_t> &indices,
                           const std::vector<float3> &faceNormals)
{
  float3 pseudo_normal{ 0.F };
  const auto vertexOffset = tri % 3;
  const auto firstVertex = tri - vertexOffset;
  auto const &curr_tri = float3{ static_cast<float>(firstVertex),
                                 static_cast<float>(firstVertex + 1),
                                 static_cast<float>(firstVertex + 2) };

  auto initVerts = [&]( ) -> std::pair<std::uint32_t, std::uint32_t> {
    std::uint32_t vert0{ };
    std::uint32_t vert1{ };
    if (res.hit_type == EDistanceType::EDGE1)
    {
      vert0 = indices[static_cast<std::size_t>(curr_tri.x)];
      vert1 = indices[static_cast<std::size_t>(curr_tri.y)];
    }
    else if (res.hit_type == EDistanceType::EDGE2)
    {
      vert0 = indices[static_cast<std::size_t>(curr_tri.y)];
      vert1 = indices[static_cast<std::size_t>(curr_tri.z)];
    }
    else if (res.hit_type == EDistanceType::EDGE3)
    {
      vert0 = indices[static_cast<std::size_t>(curr_tri.z)];
      vert1 = indices[static_cast<std::size_t>(curr_tri.x)];
    }
    return vert0 <= vert1
               ? std::pair<std::uint32_t, std::uint32_t>{ vert0, vert1 }
               : std::pair<std::uint32_t, std::uint32_t>{ vert1, vert0 };
  };

  const auto [vert0, vert1] = initVerts( );
  const EdgeIndexed edge{ vert0, vert1 };
  const auto &adjs = adjacencyMatrix.at(edge);
  assert(!adjs.empty( ));
  for (const auto &adj_tri : adjs)
  {
    const auto vertex = vert0 == adj_tri.first || vert1 == adj_tri.first
                            ? adj_tri.second
                            : adj_tri.first;
    const auto triIndex = (vertex - (vertex % 3)) / 3;

    pseudo_normal += faceNormals[triIndex];
  }
  return normalize(pseudo_normal);
}

std::vector<float> generateSignedDistanceFieldFromMesh(Mesh &mesh,
                                                       const std::uint32_t dim)
{
  const auto meshExtent = mesh.boundingBox.extent( );
  // Calculate the dimension of each voxel.
  const auto gridStep = meshExtent / static_cast<float>(dim - 1);
  // Initialise the resulting vector.
  std::vector<float> sdf(dim * dim * dim);
  assert(sdf.size( ) == dim * dim * dim);
  const auto adjacencyMatrices = getEdgeAdjacencyIndexed(mesh.indices);
  const auto faceNormals =
      generateMeshFaceNormals(mesh.indices, mesh.positions);
  const auto &numIndices = mesh.indices.size( );
  std::vector<float3> gridCoordinates(dim * dim * dim);

  auto calcDistance = [&](const float3 &voxelCoordinate) {
    float minDistance = std::numeric_limits<float>::infinity( );
    float sign = 1.f;
    // For each triangle.
    for (std::uint32_t index = 0; index < numIndices; index += 3)
    {
      const auto unsignedDistance = triangleUnsignedDistance(
          voxelCoordinate, mesh.positions[mesh.indices[index]],
          mesh.positions[mesh.indices[index + 1]],
          mesh.positions[mesh.indices[index + 2]]);

      if (std::fabs(unsignedDistance.distance) < minDistance)
      {
        minDistance = std::fabs(unsignedDistance.distance);
        // Can't be smaller than 0, so bye!!
        if (minDistance <= 1e-4F)
        {
          break;
        }
        const auto pseudoNormal =
            calculatePseudoNormal(unsignedDistance, index, mesh.positions,
                                  mesh.indices, faceNormals, adjacencyMatrices);
        const auto rayDir = voxelCoordinate - unsignedDistance.hit_point;
        sign = std::copysignf(sign, dot(rayDir, pseudoNormal));
      }
    }
    const auto xyz = voxelCoordinate / gridStep;
    const size_t voxelIndex = static_cast<size_t>(xyz.x) + dim * (static_cast<size_t>(xyz.y) + dim * static_cast<size_t>(xyz.z));
    sdf[voxelIndex] = sign * minDistance;
  };

  // Loop over z, y, x
  for (uint32_t k = 0; k < dim; ++k)
  {
    for (uint32_t j = 0; j < dim; ++j)
    {
      for (uint32_t i = 0; i < dim; ++i)
      {
        const auto voxelIndex = i + dim * (j + dim * k);
        const auto voxelCoord = gridStep * uint3{ i, j, k };
        gridCoordinates[voxelIndex] = voxelCoord;
      }
    }
  }

  std::for_each(std::execution::par_unseq,
      std::begin(gridCoordinates), std::end(gridCoordinates),
      [&](const float3 &voxelCoordinate) { calcDistance(voxelCoordinate); });
  return sdf;
}

void saveSdfAsPPMA(const std::vector<float> &sdf, const std::string &path)
{
  const auto sdfDim =
      static_cast<std::size_t>(std::floor(std::pow(sdf.size( ), 1.F / 3.F)));
  std::ofstream ppm_file;
  ppm_file.open(path);
  // Write the file header.
  ppm_file << "P3"
           << "\n";
  ppm_file << sdfDim << " " << sdfDim << "\n";
  ppm_file << "255"
           << "\n";
  auto *texture_2d = new float[sdfDim * sdfDim];
  memset(texture_2d, 0, sdfDim * sdfDim * sizeof(float));
  size_t counter = 0;
  // for (size_t iz = 0; iz < sdfDim; iz++)
  {
    for (size_t iy = 0; iy < sdfDim; iy++)
    {
      for (size_t ix = 0; ix < sdfDim; ix++)
      {
        for (size_t iz = 0; iz < sdfDim; ++iz)
        {
          const auto curr = sdf[ix + sdfDim * (iy + sdfDim * (iz))]*10000 ;
          //if(curr < 1e-4F)
          {
            texture_2d[counter] +=curr;
            break;
          }
        }
        counter++;
      }
    }
  }
  for (size_t y{ 0 }; y < sdfDim; ++y)
  {
    for (size_t x{ 0 }; x < sdfDim; ++x)
    {
      ppm_file << texture_2d[x + (sdfDim * y)] << " "
               << texture_2d[x + (sdfDim * y)] << " "
               << texture_2d[x + (sdfDim * y)] << " ";
    }
    ppm_file << "\n";
  }

  // Close the file.
  ppm_file.close( );
  delete[] texture_2d;
}

}// namespace loader
