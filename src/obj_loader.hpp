#pragma once
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <sstream>
#include <optional>

#include "structures.hpp"
/**
	 * \brief Loader for Wavefront OBJ triangle meshes
	 */

namespace loader
{
class CObjModel
{
  public:
  static std::optional<Model> LoadObjFromFile(const char *InDist);

  protected:
  /// Vertex indices used by the OBJ format
  struct SObjVertex
  {
    uint32_t p = ~0U;
    uint32_t n = ~0U;
    uint32_t uv = ~0U;

    inline SObjVertex( ) {}

    inline SObjVertex(const std::string &InString);

    inline bool operator==(const SObjVertex &v) const
    {
      return v.p == p && v.n == n && v.uv == uv;
    }
  };

  template <typename Arg, typename Result> struct unary_function
  {
    using argument_type = Arg;
    using result_type = Result;
  };
  /// Hash function for SObjVertex
  struct SObjVertexHash : unary_function<SObjVertex, size_t>
  {
    std::size_t operator( )(const SObjVertex &v) const
    {
      size_t hash = std::hash<uint32_t>( )(v.p);
      hash = hash * 37 + std::hash<uint32_t>( )(v.uv);
      hash = hash * 37 + std::hash<uint32_t>( )(v.n);
      return hash;
    }
  };
};
}