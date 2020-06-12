#pragma once
#include <vector>
#include <cstdint>

namespace loader
{

struct Vertex
{
  float px, py, pz;
  float nx, ny, nz;
  float tu, tv;
};

struct float3
{
  float3( ) = default;
  float3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
  float3(const float *in)
  {
    x = in[0];
    y = in[1];
    z = in[2];
  }
  float x, y, z;
};

struct AABB
{
  float3 min = { 1e+6, 1e+6, 1e+6 };
  float3 max = { -1e+6, -1e+6, -1e+6 };
};

struct float2
{
  float x, y;
};

struct float4
{
  float4( ) = default;
  float4(float3 a) : x(a.x), y(a.y), z(a.z), w(1.f) {}
  float4(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {}
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
  std::vector<float3> texCords;

  // 3 components for tangent and 1 for the sign to compute bitangent
  std::vector<float4> tangents;
  std::vector<float3> bitangents;
  std::vector<uint32_t> indices;

  std::vector<Primitive> primitives;

  float3 position = { 0.f, 0.f, 0.f };
  // Rotation is defined as euler angles
  float3 rotation = { 0.f, 0.f, 0.f };
  float3 scale = { 1.f, 1.f, 1.f };

  AABB boundingBox;
};

// 4 component image representation
struct Image
{
  uint16_t width, height;
  std::vector<unsigned char> image;
  const char *name;
};

struct Model
{
  std::vector<Image> images;
  std::vector<Material> materials;
  std::vector<Mesh> meshes;
};
}
