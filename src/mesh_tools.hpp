#pragma once
#include <unordered_map>
#include "structures.hpp"

#include <array>

namespace loader
{
[[nodiscard]] std::pair<float3, float3>
calculateTangentBitangent(const std::array<float3, 3> &pos,
                          const std::array<float2, 3> &tex);
bool calcBitan(const float3 &norm, const float3 &tan, float3 &bitan);
bool calcBitan(const float3 &norm, const float4 &tan, float3 &bitan);

[[nodiscard]] std::unordered_map<Vertex, std::vector<Vertex>, VertexHash> getVertexAdjacency(
    const std::vector<float3> &positions, const std::vector<float3> &normals,
    const std::vector<float2> &uvs, const std::vector<std::uint32_t> &indices);

}// namespace loader
