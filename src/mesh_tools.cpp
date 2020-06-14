#include "mesh_tools.hpp"

#include "math.h"
#include "math_helper.hpp"

#include <array>
#include <cmath>
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

  for (size_t i = 0; i < indices.size( ); i += 3)
  {
    const auto v1 = indices[i];
    const auto v2 = indices[i + 1];
    const auto v3 = indices[i + 2];

    const float3 empty{ 0.f, 0.f, 0.f };
    const float2 empty1{ 0.f, 0.f };

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

  for (size_t i = 0; i < indices.size( ); i += 3)
  {
    const auto v1 = indices[i];
    const auto v2 = indices[i + 1];
    const auto v3 = indices[i + 2];

    const float3 empty{ 0.f, 0.f, 0.f };
    const float2 empty1{ 0.f, 0.f };

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

STriangleDistanceResult
triangleUnsignedDistance(float3 from,
                         const std::array<float3, 3> &vertexPositions)
{
  float3 a = vertexPositions[0];
  float3 b = vertexPositions[1];
  float3 c = vertexPositions[2];
  auto diff = a - from;
  auto edge0 = b - a;
  auto edge1 = c - a;
  auto a00 = dot(edge0, edge0);
  auto a01 = dot(edge0, edge1);
  auto a11 = dot(edge1, edge1);
  auto b0 = dot(diff, edge0);
  auto b1 = dot(diff, edge1);
  auto det = fabsf(a00 * a11 - a01 * a01);
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

  auto pnearest = a + edge0 * s + edge1 * t;
  STriangleDistanceResult res{ };
  res.hit_type = hit_type;
  res.hit_point = pnearest;
  res.distance = (pnearest.x - from.x) * (pnearest.x - from.x) +
                 (pnearest.y - from.y) * (pnearest.y - from.y) +
                 (pnearest.z - from.z) * (pnearest.z - from.z);
  return res;
}
std::unordered_map<EdgeIndexed, std::unordered_set<EdgeIndexed,IndexedEdgeHash>, IndexedEdgeHash>
getEdgeAdjacencyIndexed(std::vector<std::uint32_t> &indices)
{
  auto vertexTable = getVertexAdjacencyIndex(indices);

  std::unordered_map<EdgeIndexed, std::unordered_set<EdgeIndexed, IndexedEdgeHash>, IndexedEdgeHash> ret;

  for (std::size_t index = 0; index < indices.size( ); index += 3)
  {
    const auto i1 = indices[index];
    const auto i2 = indices[index + 1];
    const auto i3 = indices[index + 2];

    auto parseVertex = [&](auto edge, auto vertex)
    {
      const auto arr = vertexTable[vertex];
      for(const auto& item: arr)
      {
        const EdgeIndexed place{std::min(vertex, item), std::max(vertex, item)};
        if(edge != place)
        {
          ret[edge].insert(place);
        }
      }
    };
    auto parseVertices = [&](auto edge)
    {
      parseVertex(edge, edge.first);
      parseVertex(edge, edge.second);
    };

    const EdgeIndexed e1{ std::min(i1, i2), std::max(i2, i1) };
    const EdgeIndexed e2{ std::min(i2, i3), std::max(i3, i2) };
    const EdgeIndexed e3{ std::min(i3, i1), std::max(i1, i3) };

    parseVertices(e1);
    parseVertices(e2);
    parseVertices(e3);

    //ret[e1].insert(e2);
    //ret[e1].insert(e3);
    //ret[e2].insert(e1);
    //ret[e2].insert(e3);
    //ret[e3].insert(e1);
    //ret[e3].insert(e2);
  }

  return ret;
}

}// namespace loader
