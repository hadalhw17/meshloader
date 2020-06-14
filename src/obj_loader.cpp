#include "obj_loader.hpp"

#include <algorithm>
#include <functional>
#include <thread>
#include <string>
#include <array>
#include <fmt/format.h>

namespace loader
{
template <class ArgumentType, class ResultType> struct unary_function
{
  using argument_type = ArgumentType;
  using result_type = ResultType;
};
/// Tokenize a InString into a list by splitting at 'InDelim'
std::vector<std::string> tokenize(const std::string &InString,
                                  const std::string &InDelim,
                                  bool InIncludeEmpty)
{
  std::string::size_type lastPos = 0;
  std::string::size_type pos = InString.find_first_of(InDelim, lastPos);
  std::vector<std::string> tokens;

  while (lastPos != std::string::npos)
  {
    if (pos != lastPos || InIncludeEmpty)
    {
      auto str = InString.substr(lastPos, pos - lastPos);
      str.erase(std::remove(str.begin( ), str.end( ), '-'), str.end( ));
      tokens.push_back(str);
    }
    lastPos = pos;
    if (lastPos != std::string::npos)
    {
      lastPos += 1;
      pos = InString.find_first_of(InDelim, lastPos);
    }
  }

  return tokens;
}

/// Convert a string into an unsigned integer value
auto toUInt(const std::string &InStr)
{
  char *end_ptr = nullptr;
  const auto result =
      static_cast<std::uint32_t>(std::strtoul(InStr.c_str( ), &end_ptr, 10));
  if (*end_ptr != '\0')
  {
    std::puts(
        fmt::format("Could not parse integer value: {} \n", InStr).c_str( ));
  }
  return result;
}

CObjModel::SObjVertex::SObjVertex(const std::string &InString)
{
  std::vector<std::string> tokens = tokenize(InString, "/", true);

  if (tokens.empty( ) || tokens.size( ) > 3)
  {
    std::puts(fmt::format("Invalid vertex data: {} \n", InString).c_str( ));
  }

  p = toUInt(tokens[0]);

  if (tokens.size( ) >= 2 && !tokens[1].empty( ))
  {
    uv = toUInt(tokens[1]);
  }

  if (tokens.size( ) >= 3 && !tokens[2].empty( ))
  {
    n = toUInt(tokens[2]);
  }
}

std::optional<Model>
CObjModel::LoadObjFromFile(const char *InFileName)
{
  auto model = Model( );
  model.meshes.resize(1, { });
  Mesh &mesh = model.meshes[0];

  using VertexMap = std::unordered_map<SObjVertex, uint32_t, SObjVertexHash>;

  std::ifstream is(InFileName);
  if (!is)
  {
    return std::nullopt;
  }

  std::cout << "Loading \"" << InFileName << "\" .. " << std::endl;

  std::vector<float3> positions;
  std::vector<float2> texcoords;
  std::vector<float3> normals;
  std::vector<SObjVertex> vertices;
  VertexMap vertexMap;

  std::string lineStr;
  while (std::getline(is, lineStr))
  {
    std::istringstream line(lineStr);

    std::string prefix;
    line >> prefix;

    if (prefix == "v")
    {
      float3 p{ };
      line >> p.x >> p.y >> p.z;
      positions.push_back(p);
    }
    else if (prefix == "vt")
    {
      float2 tc{ };
      line >> tc.x >> tc.y;
      texcoords.push_back(tc);
    }
    else if (prefix == "vn")
    {
      float3 n{ };
      line >> n.x >> n.y >> n.z;
      normals.push_back((n));
    }
    else if (prefix == "f")
    {
      std::array<std::string, 4> v;
      line >> v[0] >> v[1] >> v[2] >> v[3];
      std::array<SObjVertex, 6> verts;
      std::size_t nVertices = 3;

      verts[0] = SObjVertex(v[0]);
      verts[1] = SObjVertex(v[1]);
      verts[2] = SObjVertex(v[2]);

      if (!v[3].empty( ))
      {
        /* This is a quad, split into two triangles */
        verts[3] = SObjVertex(v[3]);
        verts[4] = verts[0];
        verts[5] = verts[2];
        nVertices = verts.size( );
      }
      /* Convert to an indexed vertex list */
      for (size_t i = 0; i < nVertices; ++i)
      {
        const SObjVertex &vertex = verts.at(i);
        const auto it = vertexMap.find(vertex);
        if (it == vertexMap.end( ))
        {
          vertexMap[vertex] = static_cast<std::uint32_t>(vertices.size( ));
          mesh.indices.push_back(static_cast<std::uint32_t>(vertices.size( )));
          vertices.push_back(vertex);
        }
        else
        {
          mesh.indices.push_back(it->second);
        }
      }
    }
  }

  mesh.positions.resize(vertices.size( ));
  if (!texcoords.empty( ))
  {
    mesh.texCords.resize(vertices.size( ));
  }
  if (!normals.empty( ))
  {
    mesh.normals.resize(vertices.size( ));
  }

  auto copyPositions = [&positions, &vertices](std::vector<float3> &InDist) {
    for (size_t i = 0; i < InDist.size( ); ++i)
    {
      InDist[i] = positions[vertices[i].p - 1];
    }
  };

  auto copyNorms = [&normals, &vertices](std::vector<float3> &InDist) {
    for (size_t i = 0; i < InDist.size( ); ++i)
    {
      InDist[i] = normals[vertices[i].n - 1];
    }
  };

  auto copyTexCords = [&texcoords, &vertices](std::vector<float2> &InDist) {
    for (size_t i = 0; i < InDist.size( ); ++i)
    {
      InDist[i] = texcoords[vertices[i].uv - 1];
    }
  };

#ifdef NO_MT
  copyPositions(std::ref(mesh.positions));
  copyNorms(std::ref(mesh.normals));
  copyTexCords(std::ref(mesh.texCords));
#else
  std::thread posLoader(copyPositions, std::ref(mesh.positions));
  std::thread normLoader(copyNorms, std::ref(mesh.normals));
  std::thread uvLoader(copyTexCords, std::ref(mesh.texCords));

  posLoader.join( );
  normLoader.join( );
  uvLoader.join( );
#endif
  return model;
}
}