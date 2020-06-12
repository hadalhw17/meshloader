#pragma once
#include "structures.hpp"

namespace loader
{
float3 cross(float3 a, float3 b)
{
  float3 ret = { };

  ret.x = a.y * b.z - b.y * a.z;
  ret.y = a.z * b.x - b.z * a.x;
  ret.z = a.x * b.y - b.x * a.y;

  return ret;
}
}
