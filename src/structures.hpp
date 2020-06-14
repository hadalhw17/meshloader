#pragma once
#include <compare>
#include <cstdint>
#include <fmt/format.h>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace loader
{

struct float3
{
  float3( ) = default;
  float3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
  explicit float3(float _x) : x(_x), y(_x), z(_x) {}
  explicit float3(const float *in)
  {
    x = in[0];
    y = in[1];
    z = in[2];
  }

  auto operator<=>(const float3 &rhs) const = default;
  [[nodiscard]] auto operator+(const float3 &rhs) const
  {
    return float3{ x + rhs.x, y + rhs.y, z + rhs.z };
  }
  [[nodiscard]] auto operator+(const float rhs) const
  {
    return float3(rhs) + *this;
  }
  [[nodiscard]] auto operator-(const float3 &rhs) const
  {
    return float3{ x - rhs.x, y - rhs.y, z - rhs.z };
  }
  [[nodiscard]] auto operator*(const float3 &rhs) const
  {
    return float3{ x * rhs.x, y * rhs.y, z * rhs.z };
  }
  [[nodiscard]] auto operator*(const float rhs) const
  {
    return float3(rhs) * (*this);
  }
  [[nodiscard]] auto operator/(const float3 &rhs) const
  {
    return float3{ x / rhs.x, y / rhs.y, z / rhs.z };
  }
  [[nodiscard]] auto operator/(const float rhs) const
  {
    return float3{ x / rhs, y / rhs, z / rhs };
  }
  auto &operator+=(const float3 &rhs)
  {
    *this = *this + rhs;
    return *this;
  }
  friend std::ostream &operator<<(std::ostream &, const float3 &);

  float x, y, z;
};

struct AABB
{
  float3 min{ 1e+6F, 1e+6F, 1e+6F };
  float3 max{ -1e+6F, -1e+6F, -1e+6F };
};

struct float2
{
  float x, y;

  float2( ) = default;
  float2(float _x, float _y) : x(_x), y(_y) {}
  float2(const float *in)
  {
    x = in[0];
    y = in[1];
  }
  auto operator<=>(const float2 &) const = default;
  friend std::ostream &operator<<(std::ostream &, const float2 &);
};

struct float4
{
  float4( ) = default;
  explicit float4(float3 a) : x(a.x), y(a.y), z(a.z), w(1.f) {}
  float4(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {}
  explicit float4(const float *in)
  {
    x = in[0];
    y = in[1];
    z = in[2];
    w = in[3];
  }
  float x, y, z, w;
};

struct Material
{
  int albedoTexture;
  float4 baseContribution;
  int normalTexture;
  float normalScale;
  int metallicRoughnessTexture;
  float roughness;
  float metallic;

  int occlusionTexture;
  int emissiveTexture;
  float3 emitFactor;
};

struct Primitive
{
  std::uint32_t firstIndex;
  std::uint32_t indexCount;

  int material;
};

struct Mesh
{
  std::vector<float3> positions;
  std::vector<float3> normals;
  std::vector<float2> texCords;

  // 3 components for tangent and 1 for the sign to compute bitangent
  std::vector<float4> tangents;
  std::vector<float3> bitangents;
  std::vector<std::uint32_t> indices;

  std::vector<Primitive> primitives;

  float3 position{ 0.f, 0.f, 0.f };
  // Rotation is defined as euler angles
  float3 rotation{ 0.f, 0.f, 0.f };
  float3 scale{ 1.f, 1.f, 1.f };

  AABB boundingBox;
};

// 4 component image representation
struct Image
{
  std::uint32_t width, height;
  std::vector<unsigned char> image;
  const char *name;
};

struct Model
{
  std::vector<Image> images;
  std::vector<Material> materials;
  std::vector<Mesh> meshes;
};

enum class EDistanceType
{
  FACE = 0, // Hit in the face.
  EDGE1 = 1,// Hit on the first edge.
  EDGE2 = 2,// Hit on the second edge.
  EDGE3 = 3,// Hit on the third edge.
  VERT1 = 4,// Hit on the first vertex.
  VERT2 = 5,// Hit on the second vertex.
  VERT3 = 6 // Hit on the third vertex.
};

struct STriangleDistanceResult
{
  float distance;
  EDistanceType hit_type;
  float3 hit_point;
};

struct Vertex
{
  float3 position;
  float3 normal;
  float2 texCord;

  auto operator<=>(const Vertex &rhs) const = default;
  friend std::ostream &operator<<(std::ostream &, const Vertex &);
};
inline std::ostream &operator<<(std::ostream &os, const float2 &fl)
{
  const auto res = fmt::format("{{ x: {}, y: {}}}", fl.x, fl.y);
  return os << res << std::endl;
}
inline std::ostream &operator<<(std::ostream &os, const float3 &fl)
{
  const auto res = fmt::format("{{ x: {}, y: {}, z: {}}}", fl.x, fl.y, fl.z);
  return os << res << std::endl;
}

inline std::ostream &operator<<(std::ostream &os, const Vertex &vert)
{
  return os << "position " << vert.position << " normal " << vert.normal
            << " texture coordinate " << vert.texCord << std::endl;
}

struct VertexHash
{
  std::size_t operator( )(const Vertex &vertex) const
  {
    return static_cast<std::uint32_t>(vertex.position.x * 19965295109.F) ^
           static_cast<std::uint32_t>(vertex.position.y * 18511065037.F) ^
           static_cast<std::uint32_t>(vertex.position.z * 45183875657.F) ^
           static_cast<std::uint32_t>(vertex.normal.x * 34699057009.F) ^
           static_cast<std::uint32_t>(vertex.normal.y * 56587034149.F) ^
           static_cast<std::uint32_t>(vertex.normal.z * 79652433737.F) ^
           static_cast<std::uint32_t>(vertex.texCord.x * 13739426149.F) ^
           static_cast<std::uint32_t>(vertex.texCord.y * 59901554101.F);
  }
};

using Edge = std::pair<Vertex, Vertex>;
using EdgeIndexed = std::pair<std::uint32_t, std::uint32_t>;

struct IndexedEdgeHash
{
  std::size_t operator( )(const EdgeIndexed &edge) const
  {
    auto pack = [](const uint32_t &v1, const uint32_t &v2) {
      uint64_t r = v1;
      r <<= 32;
      r |= v2;
      return r;
    };
    std::uint64_t packed = pack(edge.first, edge.second);
    std::uint64_t v = packed * 3935559000370003845 + 2691343689449507681;

    v ^= v >> 21;
    v ^= v << 37;
    v ^= v >> 4;

    v *= 4768777513237032717;

    v ^= v << 20;
    v ^= v >> 41;
    v ^= v << 5;
    return v;
  }
};

struct EdgeHash
{
  std::size_t operator( )(const Edge &edge) const
  {
    VertexHash hash;
    return hash(edge.first) ^ hash(edge.second);
  }
};

}// namespace loader
