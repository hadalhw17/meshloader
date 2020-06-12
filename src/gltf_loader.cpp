#include "gltf_loader.hpp"

#include <vector>

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "mesh_tools.hpp"
#include "tiny_gltf.h"

#include <fmt/format.h>
#include <iostream>

namespace loader
{
constexpr float PI = 3.14159265358979323F /* pi */;
constexpr float PI_2 = PI / 2.F;
constexpr float TO_DEG = 180.F / PI;
using namespace tinygltf;

float3 toEulerAngles(const std::array<float, 4> &quaternion)
{
  float3 angles = { };
  const auto sinr_cosp =
      2.F * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2]);
  const auto cosr_cosp = 1.F - 2.F * (quaternion[0] * quaternion[0] +
                                      quaternion[1] * quaternion[1]);
  angles.x = atan2f(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  const auto sinp =
      2.F * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0]);
  if (std::fabs(sinp) >= 1)
  {
    angles.y =
        std::copysignf(PI_2, sinp);// use 90 degrees if out of range
  }
  else
  {
    angles.y = std::asin(sinp);
  }

  // yaw (z-axis rotation)
  const auto siny_cosp =
      2.F * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]);
  const auto cosy_cosp =
      1.F - 2.F * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
  angles.z = std::atan2(siny_cosp, cosy_cosp);

  angles.x *= TO_DEG;
  angles.y *= TO_DEG;
  angles.z *= TO_DEG;
  return angles;
}

template <typename T>
void getAttribute(std::vector<T> &attribute, tinygltf::Model &gltfMesh, tinygltf::Primitive &prim,
                  const char *attrubuteName)
{
  const auto &posAccess = gltfMesh.accessors[static_cast<unsigned long>(
      prim.attributes[attrubuteName])];
  if (posAccess.count <= 0)
  {
    return;
  }
  attribute.resize(posAccess.count);
  const auto &posView =
      gltfMesh.bufferViews[static_cast<unsigned long>(posAccess.bufferView)];
  const auto &posBuffer =
      gltfMesh.buffers[static_cast<unsigned long>(posView.buffer)];
  const auto *pos = reinterpret_cast<const float *>(
      &(posBuffer.data[posAccess.byteOffset + posView.byteOffset]));
  const auto stride =
      static_cast<unsigned long>(posAccess.ByteStride(posView)) / sizeof(float);
  for (size_t a = 0; a < posAccess.count; ++a)
  {
    T fPos = T(&pos[a * stride]);
    attribute[a] = fPos;
  }
}

void loadMaterials(std::vector<loader::Material> &dist,
                   const std::vector<tinygltf::Material> &src,
                   const std::vector<Texture> &textures)
{
  for (const auto &sMat : src)
  {
    loader::Material mat{ };
    const auto albedo = sMat.pbrMetallicRoughness.baseColorTexture.index;
    mat.albedoTexture =
        albedo == -1 ? -1 : textures[static_cast<unsigned long>(albedo)].source;
    mat.baseContribution = {
      static_cast<float>(sMat.pbrMetallicRoughness.baseColorFactor[0]),
      static_cast<float>(sMat.pbrMetallicRoughness.baseColorFactor[1]),
      static_cast<float>(sMat.pbrMetallicRoughness.baseColorFactor[2]),
      static_cast<float>(sMat.pbrMetallicRoughness.baseColorFactor[3]),
    };
    const auto normal = sMat.normalTexture.index;
    mat.normalTexture =
        normal == -1 ? -1 : textures[static_cast<unsigned long>(normal)].source;
    mat.normalScale = static_cast<float>(sMat.normalTexture.scale);
    const auto emissionTex = sMat.emissiveTexture.index;
    mat.emissiveTexture =
        emissionTex == -1
            ? -1
            : textures[static_cast<unsigned long>(emissionTex)].source;
    mat.emitFactor = float3(static_cast<float>(sMat.emissiveFactor[0]),
                            static_cast<float>(sMat.emissiveFactor[1]),
                            static_cast<float>(sMat.emissiveFactor[2]));
    const auto metRough =
        sMat.pbrMetallicRoughness.metallicRoughnessTexture.index;
    mat.metallicRoughnessTexture =
        metRough == -1 ? -1
                       : textures[static_cast<unsigned long>(metRough)].source;
    const auto occlusuion = sMat.occlusionTexture.index;
    mat.occlusionTexture =
        occlusuion == -1
            ? -1
            : textures[static_cast<unsigned long>(occlusuion)].source;
    mat.roughness =
        static_cast<float>(sMat.pbrMetallicRoughness.roughnessFactor);
    mat.metallic = static_cast<float>(sMat.pbrMetallicRoughness.metallicFactor);
    dist.emplace_back(mat);
  }
}

void loadTextures(std::vector<loader::Image> &InDst, std::vector<tinygltf::Image> &InSrc)
{
  for (auto &sImg : InSrc)
  {
    InDst.emplace_back(loader::Image( ));
    loader::Image &img = InDst.back( );
    img.height = static_cast<uint16_t>(sImg.height);
    img.width = static_cast<uint16_t>(sImg.width);
    if (sImg.component != 4)
    {
      img.image.resize(img.width * img.height * 4U);
      for (auto i = 0U; i < sImg.image.size( ); i += 3)
      {
        img.image.push_back(sImg.image[i]);
        img.image.push_back(sImg.image[i + 1]);
        img.image.push_back(sImg.image[i + 2]);
        img.image.push_back('1');
      }
    }
    else
    {
      img.image = std::move(sImg.image);
    }
  }
}

std::optional<loader::Model> loadGlTf(const char *InPath)
{

  TinyGLTF loader;
  tinygltf::Model gltfMesh;
  std::string err;
  std::string warn;

  bool ret = loader.LoadASCIIFromFile(&gltfMesh, &err, &warn, InPath);

  if (!err.empty( ))
  {
    std::puts(fmt::format("glTF error: {}", err.c_str( )).c_str( ));
  }

  if (!warn.empty( ))
  {
    std::puts(fmt::format("glTF warning: {}", warn.c_str( )).c_str( ));
  }

  if (!ret)
  {
    std::puts(
        fmt::format("glTF failed to load model at InPath {}; \nTerminating",
                    InPath)
            .c_str( ));
    return std::nullopt;
  }

  auto model = loader::Model( );

  loadMaterials(model.materials, gltfMesh.materials, gltfMesh.textures);
  loadTextures(model.images, gltfMesh.images);
  model.meshes.resize(gltfMesh.meshes.size( ), { });

  size_t i = 0;
  for (auto &mesh : model.meshes)
  {
    auto &tinyMesh = gltfMesh.meshes[i];
    for (auto &prim : tinyMesh.primitives)
    {
      auto hasTangents = true;
      const auto vertexStart = mesh.positions.size( );
      getAttribute(mesh.positions, gltfMesh, prim, "POSITION");
      getAttribute(mesh.tangents, gltfMesh, prim, "TANGENT");
      getAttribute(mesh.normals, gltfMesh, prim, "NORMAL");
      getAttribute(mesh.texCords, gltfMesh, prim, "TEXCOORD_0");
      mesh.bitangents.resize(mesh.positions.size( ));
      if (mesh.tangents.size( ) != mesh.positions.size( ))
      {
        hasTangents = false;
        mesh.tangents.resize(mesh.positions.size( ));
      }

      bool hasIndices = prim.indices > -1;
      if (hasIndices)
      {
        const tinygltf::Accessor &indexAccessor =
            gltfMesh.accessors[static_cast<unsigned long>(prim.indices)];
        const tinygltf::BufferView &bufferView =
            gltfMesh.bufferViews[static_cast<unsigned long>(
                indexAccessor.bufferView)];
        const tinygltf::Buffer &buffer =
            gltfMesh.buffers[static_cast<unsigned long>(bufferView.buffer)];

        const auto indexCount = static_cast<uint32_t>(indexAccessor.count);
        mesh.primitives.push_back(
            { static_cast<std::uint32_t>(mesh.indices.size( )), indexCount,
              prim.material });

        const void *dataPtr =
            &(buffer.data[indexAccessor.byteOffset + bufferView.byteOffset]);

        auto tanBitan = [&mesh, hasTangents](auto index) {
          if ((index + 1) % 3 == 0)
          {
            if (!hasTangents)
            {

              const auto ind = mesh.indices[mesh.indices.size( ) - 1];
              const auto ind1 = mesh.indices[mesh.indices.size( ) - 2];
              const auto ind2 = mesh.indices[mesh.indices.size( ) - 3];
              const std::array<float3, 3> pos = { mesh.positions[ind2],
                                                  mesh.positions[ind1],
                                                  mesh.positions[ind] };
              const std::array<float3, 3> tex = { mesh.texCords[ind2],
                                                  mesh.texCords[ind1],
                                                  mesh.texCords[ind] };

              auto [tan, bitan] = calculateTangentBitangent(pos, tex);
              mesh.tangents[ind2] = tan;
              mesh.bitangents[ind2] = bitan;

              mesh.tangents[ind1] = tan;
              mesh.bitangents[ind1] = bitan;

              mesh.tangents[ind] = tan;
              mesh.bitangents[ind] = bitan;
            }
            else
            {
              const auto ind = mesh.indices[mesh.indices.size( ) - 1];
              const auto ind1 = mesh.indices[mesh.indices.size( ) - 2];
              const auto ind2 = mesh.indices[mesh.indices.size( ) - 3];
              const std::array<float3, 3> norm = { mesh.normals[ind2],
                                                   mesh.normals[ind1],
                                                   mesh.normals[ind] };
              const std::array<float4, 3> tan = { mesh.tangents[ind2],
                                                  mesh.tangents[ind1],
                                                  mesh.tangents[ind] };
              calcBitan(norm[0], tan[0], mesh.bitangents[ind2]);
              calcBitan(norm[0], tan[0], mesh.bitangents[ind1]);
              calcBitan(norm[0], tan[0], mesh.bitangents[ind]);
            }
          }
        };

        auto loadIndices = [&](auto templ) {
          using IndexType = decltype(templ);
          mesh.indices.resize(indexCount);
          const auto *buf = static_cast<const IndexType *>(dataPtr);
          for (size_t index = 0; auto &ind : mesh.indices)
          {
            ind = buf[index] + static_cast<IndexType>(vertexStart);
            tanBitan(index);
            ++index;
          }
        };
        switch (indexAccessor.componentType)
        {
        case TINYGLTF_PARAMETER_TYPE_UNSIGNED_INT:
        {
          std::uint32_t templ{ };
          loadIndices(templ);
          break;
        }
        case TINYGLTF_PARAMETER_TYPE_UNSIGNED_SHORT:
        {
          std::uint16_t templ{ };
          loadIndices(templ);
          break;
        }
        case TINYGLTF_PARAMETER_TYPE_UNSIGNED_BYTE:
        {
          std::uint8_t templ{ };
          loadIndices(templ);
          break;
        }
        default:
          std::cerr << "Index component type " << indexAccessor.componentType
                    << " not supported!" << std::endl;
          return std::nullopt;
        }
      }
    }

    ++i;
  }

  for (const auto &node : gltfMesh.nodes)
  {
    if (node.children.empty( ) && node.mesh >= 0)
    {
      loader::Mesh &mesh = model.meshes[static_cast<unsigned long>(node.mesh)];
      const std::vector<double> &translation = node.translation;
      const std::vector<double> &scale = node.scale;
      const std::vector<double> &rotation = node.rotation;
      if (!translation.empty( ))
      {
        mesh.position = float3(static_cast<float>(node.translation[0]),
                               static_cast<float>(node.translation[1]),
                               static_cast<float>(node.translation[2]));
      }

      if (!scale.empty( ))
      {
        mesh.scale = float3(static_cast<float>(node.scale[0]),
                            static_cast<float>(node.scale[1]),
                            static_cast<float>(node.scale[2]));
      }
      if (rotation.size( ) == 4)
      {
        mesh.rotation = toEulerAngles({ static_cast<float>(node.rotation[0]),
                                        static_cast<float>(node.rotation[1]),
                                        static_cast<float>(node.rotation[2]),
                                        static_cast<float>(node.rotation[3]) });
      }
    }
  }
  return std::optional(std::reference_wrapper(model));
}
}
