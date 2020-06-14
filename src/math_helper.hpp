#pragma once
#include "structures.hpp"

namespace loader
{
[[nodiscard]] inline float3 cross(float3 a, float3 b)
{
  float3 ret = { };

  ret.x = a.y * b.z - b.y * a.z;
  ret.y = a.z * b.x - b.z * a.x;
  ret.z = a.x * b.y - b.x * a.y;

  return ret;
}

[[nodiscard]] inline float dot(float3 a, float3 b)
{
  return a.x * b.x + a.x * b.y + a.z * b.z;
}

[[nodiscard]] inline float3 normalize(const float3 a)
{
  float aDotA = dot(a, a);
  return a / sqrt(aDotA);
}
}// namespace loader
