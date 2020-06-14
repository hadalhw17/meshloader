#ifndef _EDGE_TOOLS__
#define _EDGE_TOOLS__
#include "structures.hpp"

#include <array>
#include <unordered_map>
#include <unordered_set>

namespace loader
{
template <typename IndexType>
using VertexAdjacencyMatrix =
    std::unordered_map<IndexType, std::vector<IndexType>>;

using EdgeAdjacencyMatrix =
    std::unordered_map<EdgeIndexed,
                       std::unordered_set<EdgeIndexed, IndexedEdgeHash>,
                       IndexedEdgeHash>;

[[nodiscard]] float3 calculateFaceNormal(const std::array<float3, 3> face);
[[nodiscard]] std::pair<float3, float3>
calculateTangentBitangent(const std::array<float3, 3> &pos,
                          const std::array<float2, 3> &tex);
bool calcBitan(const float3 &norm, const float3 &tan, float3 &bitan);
bool calcBitan(const float3 &norm, const float4 &tan, float3 &bitan);

[[nodiscard]] std::unordered_map<Vertex, std::vector<Vertex>, VertexHash>
getVertexAdjacency(const std::vector<float3> &positions,
                   const std::vector<float3> &normals,
                   const std::vector<float2> &uvs,
                   const std::vector<std::uint32_t> &indices);

template <typename IndexType>
[[nodiscard]] inline VertexAdjacencyMatrix<IndexType>
getVertexAdjacencyIndex(std::vector<IndexType> &indices)
{
  std::unordered_map<IndexType, std::vector<IndexType>> ret;
  for (std::size_t index = 0; index < indices.size( ); index += 3)
  {
    const auto i1 = indices[index];
    const auto i2 = indices[index + 1];
    const auto i3 = indices[index + 2];

    ret[i1].push_back(i2);
    ret[i1].push_back(i3);
    ret[i2].push_back(i1);
    ret[i2].push_back(i3);
    ret[i3].push_back(i1);
    ret[i3].push_back(i2);
  }
  return ret;
}

[[nodiscard]] std::unordered_map<Edge, std::vector<Edge>, EdgeHash>
getEdgeAdjacency(const std::vector<float3> &positions,
                 const std::vector<float3> &normals,
                 const std::vector<float2> &uvs,
                 const std::vector<std::uint32_t> &indices);

[[nodiscard]] std::pair<VertexAdjacencyMatrix<std::uint32_t>,
                        EdgeAdjacencyMatrix>
getEdgeAdjacencyIndexed(std::vector<std::uint32_t> &indices);

[[nodiscard]] STriangleDistanceResult
triangleUnsignedDistance(float3 from,
                         const std::array<float3, 3> &vertexPositions);

[[nodiscard]] float3 calculatePseudoNormal(
    const STriangleDistanceResult &distanceRes, const std::uint32_t index,
    const std::vector<float3> &positions, const std::vector<uint32_t> &indices,
    const std::pair<VertexAdjacencyMatrix<uint32_t>, EdgeAdjacencyMatrix>
        &adjacency);

[[nodiscard]] float3 getPseudoNormalVertex(
    const std::uint32_t v_nearest,
    const VertexAdjacencyMatrix<std::uint32_t> &vertexAdjacencyMatrix,
    const std::vector<float3> &positions);

[[nodiscard]] float3
getPseudoNormalEdge(const STriangleDistanceResult &res, const std::uint32_t tri,
                    const EdgeAdjacencyMatrix &adjacencyMatrix,
                    const std::vector<float3> &positions);
}// namespace loader
#endif
