#include "mesh_tools.hpp"

#include "math_helper.hpp"

#include <array>
#include <cmath>

namespace loader
{
std::pair<float3, float3>
calculateTangentBitangent(const std::array<float3, 3> &pos,
                          const std::array<float3, 3> &tex)
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
}
