#include "mesh_tools.hpp"

#include "math_helper.hpp"

#include <array>
#include <cmath>
#include <iostream>

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
std::unordered_multimap<Vertex, Vertex, VertexHash> getVertexAdjacency(
    const std::vector<float3> &positions, const std::vector<float3> &normals,
    const std::vector<float3> &uvs, const std::vector<std::uint32_t> &indices)
{
  std::unordered_multimap<Vertex, Vertex, VertexHash> ret;
  bool hasNormals = !normals.empty();
  bool hasUvs = !uvs.empty();

  for (size_t i = 0; i < indices.size( ); i += 3)
  {
    const auto v1 = indices[i];
    const auto v2 = indices[i + 1];
    const auto v3 = indices[i + 2];

    const float3 empty{0.f, 0.f, 0.f};

    Vertex vertex1{ .position{ positions[v1] },
                    .normal{ hasNormals ? normals[v1] : empty },
                    .texCord{ hasUvs ? uvs[v1] : empty } };

    Vertex vertex2{ .position{ positions[v2] },
                    .normal{ hasNormals ?  normals[v2] : empty },
                    .texCord{ hasUvs ? uvs[v2] : empty } };

    Vertex vertex3{ .position{ positions[v3] },
                    .normal{ hasNormals ? normals[v3] : empty },
                    .texCord{ hasUvs ? uvs[v3] : empty } };
    ret.insert({vertex1, vertex2});
    ret.insert({vertex1, vertex3});
    ret.insert({vertex2, vertex1});
    ret.insert({vertex2, vertex3});
    ret.insert({vertex3, vertex1});
    ret.insert({vertex3, vertex2});
  }

  return ret;
}

// TriangleDistanceResult unsigned_distance_triangle(float3 p, int tri_index)
//{
//  TriangleDistanceResult res;
//  DistanceType hit_type;
//
//  float3 tri = mesh->faces[tri_index];
//  float3 a = mesh->verts[(size_t)tri.x];
//  float3 b = mesh->verts[(size_t)tri.y];
//  float3 c = mesh->verts[(size_t)tri.z];
//  auto diff = a - p;
//  auto edge0 = b - a;
//  auto edge1 = c - a;
//  auto a00 = dot(edge0, edge0);
//  auto a01 = dot(edge0, edge1);
//  auto a11 = dot(edge1, edge1);
//  auto b0 = dot(diff, edge0);
//  auto b1 = dot(diff, edge1);
//  auto det = fabsf(a00 * a11 - a01 * a01);
//  auto s = a01 * b1 - a11 * b0;
//  auto t = a01 * b0 - a00 * b1;
//
//  if (s + t <= det)
//  {
//    if (s < 0)
//    {
//      if (t < 0) // region 4
//      {
//        if (b0 < 0)
//        {
//          t = 0;
//          if (-b0 >= a00)
//          {
//            // VN1
//            hit_type = DistanceType::VERT2;
//            s = 1;
//          }
//          else
//          {
//            // EN0
//            hit_type = DistanceType::EDGE1;
//            s = -b0 / a00;
//          }
//        }
//        else
//        {
//          s = 0;
//
//          if (b1 >= 0)
//          {
//            // VN0
//            hit_type = DistanceType::VERT1;
//            t = 0;
//          }
//          else if (-b1 >= a11)
//          {
//            // VN2
//            hit_type = DistanceType::VERT3;
//            t = 1;
//          }
//          else
//          {
//            // EN2
//            hit_type = DistanceType::EDGE3;
//            t = -b1 / a11;
//          }
//        }
//      }
//      else // region 3
//      {
//        s = 0;
//
//        if (b1 >= 0)
//        {
//          // VN0
//          hit_type = DistanceType::VERT1;
//          t = 0;
//        }
//        else if (-b1 >= a11)
//        {
//          // VN2
//          hit_type = DistanceType::VERT3;
//          t = 1;
//        }
//        else
//        {
//          // EN2
//          hit_type = DistanceType::EDGE3;
//          t = -b1 / a11;
//        }
//      }
//    }
//    else if (t < 0) // region 5
//    {
//      t = 0;
//
//      if (b0 >= 0)
//      {
//        // VN0
//        hit_type = DistanceType::VERT1;
//        s = 0;
//      }
//      else if (-b0 >= a00)
//      {
//        // VN1
//        hit_type = DistanceType::VERT2;
//        s = 1;
//      }
//      else
//      {
//        // EN0
//        hit_type = DistanceType::EDGE1;
//        s = -b0 / a00;
//      }
//    }
//    else // region 0
//    {
//      // FN
//      hit_type = DistanceType::FACE;
//      // minimum at interior point
//      auto invDet = (1) / det;
//      s *= invDet;
//      t *= invDet;
//    }
//  }
//  else
//  {
//    float tmp0, tmp1, numer, denom;
//
//    if (s < 0) // region 2
//    {
//      tmp0 = a01 + b0;
//      tmp1 = a11 + b1;
//
//      if (tmp1 > tmp0)
//      {
//        numer = tmp1 - tmp0;
//        denom = a00 - (2) * a01 + a11;
//
//        if (numer >= denom)
//        {
//          // VN1
//          hit_type = DistanceType::VERT2;
//          s = 1;
//          t = 0;
//        }
//        else
//        {
//          // EN1
//          hit_type = DistanceType::EDGE2;
//          s = numer / denom;
//          t = 1 - s;
//        }
//      }
//      else
//      {
//        s = 0;
//
//        if (tmp1 <= 0)
//        {
//          // VN2
//          hit_type = DistanceType::VERT3;
//          t = 1;
//        }
//        else if (b1 >= 0)
//        {
//          // VN0
//          hit_type = DistanceType::VERT1;
//          t = 0;
//        }
//        else
//        {
//          // EN2
//          hit_type = DistanceType::EDGE3;
//          t = -b1 / a11;
//        }
//      }
//    }
//    else if (t < 0) // region 6
//    {
//      tmp0 = a01 + b1;
//      tmp1 = a00 + b0;
//
//      if (tmp1 > tmp0)
//      {
//        numer = tmp1 - tmp0;
//        denom = a00 - 2 * a01 + a11;
//
//        if (numer >= denom)
//        {
//          // VN2
//          hit_type = DistanceType::VERT3;
//          t = 1;
//          s = 0;
//        }
//        else
//        {
//          // EN1
//          hit_type = DistanceType::EDGE2;
//          t = numer / denom;
//          s = 1 - t;
//        }
//      }
//      else
//      {
//        t = 0;
//
//        if (tmp1 <= 0)
//        {
//          // VN1
//          hit_type = DistanceType::VERT2;
//          s = 1;
//        }
//        else if (b0 >= 0)
//        {
//          // VN0
//          hit_type = DistanceType::VERT1;
//          s = 0;
//        }
//        else
//        {
//          // EN0
//          hit_type = DistanceType::EDGE1;
//          s = -b0 / a00;
//        }
//      }
//    }
//    else // region 1
//    {
//      numer = a11 + b1 - a01 - b0;
//
//      if (numer <= 0)
//      {
//        // VN2
//        hit_type = DistanceType::VERT3;
//        s = 0;
//        t = 1;
//      }
//      else
//      {
//        denom = a00 - (2) * a01 + a11;
//
//        if (numer >= denom)
//        {
//          // VN1
//          hit_type = DistanceType::VERT2;
//          s = 1;
//          t = 0;
//        }
//        else
//        {
//          // EN1
//          hit_type = DistanceType::EDGE2;
//          s = numer / denom;
//          t = 1 - s;
//        }
//      }
//    }
//  }
//
//  auto pnearest = a + edge0 * s + edge1 * t;
//  auto vec = pnearest - p;
//  res.hit_type = hit_type;
//  res.hit_point = pnearest;
//  res.distance = (pnearest.x - p.x) * (pnearest.x - p.x) + (pnearest.y - p.y)
//  * (pnearest.y - p.y) + (pnearest.z - p.z) * (pnearest.z - p.z); return res;
//}

}// namespace loader
