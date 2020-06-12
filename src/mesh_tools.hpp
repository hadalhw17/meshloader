#pragma once
#include "structures.hpp"

#include <array>

namespace loader
{
[[nodiscard]] std::pair<float3, float3>
calculateTangentBitangent(const std::array<float3, 3> &pos,
                          const std::array<float3, 3> &tex);
bool calcBitan(const float3 &norm, const float3 &tan, float3 &bitan);
bool calcBitan(const float3 &norm, const float4 &tan, float3 &bitan);
}
