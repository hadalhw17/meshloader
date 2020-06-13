#include "../src/mesh_loader.hpp"
#include "../src/mesh_tools.hpp"
#include "config.hpp"
#include "fmt/format.h"

#include <algorithm>
#include <catch2/catch.hpp>
#include <iostream>
#include <optional>

TEST_CASE("Loading valid obj model", "[obj]")
{
  auto model = loader::loadMesh(
      fmt::format("{}{}", APP_PATH, "/test/dragon.obj").c_str( ));
  REQUIRE(model.has_value( ));
  REQUIRE(model.value( ).meshes.size( ) == 1);
  REQUIRE(model.value( ).meshes[0].positions.size( ) ==
          model.value( ).meshes[0].normals.size( ));
  REQUIRE(model.value( ).meshes[0].indices.size( ) >
          model.value( ).meshes[0].positions.size( ));
}

TEST_CASE("Loading valid glTF model", "[glTF]")
{
  auto model = loader::loadMesh(
      fmt::format("{}{}", APP_PATH, "/test/cube_gltf/Cube.gltf").c_str( ));
  REQUIRE(model.has_value( ));
  REQUIRE(model.value( ).meshes[0].positions.size( ) == 36);
  REQUIRE(model.value( ).meshes[0].normals.size( ) == 36);
  REQUIRE(model.value( ).meshes[0].texCords.size( ) == 36);
  REQUIRE(model.value( ).meshes[0].primitives[0].indexCount ==
          model.value( ).meshes[0].indices.size( ));
  REQUIRE(model.value( ).images[0].image.size( ) ==
          model.value( ).images[0].width * model.value( ).images[0].height * 4);
  REQUIRE(model.value( ).images[1].image.size( ) ==
          model.value( ).images[1].width * model.value( ).images[1].height * 4);
}

TEST_CASE("Loading invalid obj model", "[obj]")
{
  auto model = loader::loadMesh("asdfewasdfesdf");
  REQUIRE(!model.has_value( ));
}

TEST_CASE("Loading invalid glTF model", "[glTF]")
{
  auto model = loader::loadMesh("asdajkgsdfnksjldfbnlekjnsd");
  REQUIRE(!model.has_value( ));
}

TEST_CASE("Testing vertex adjacency list", "[vert_adj]")
{
  auto model = loader::loadMesh(
      fmt::format("{}{}", APP_PATH, "/test/cube_gltf/Cube.gltf").c_str( ));
  auto &mesh = model.value( ).meshes[0];
  auto adjList = loader::getVertexAdjacency(mesh.positions, mesh.normals,
                                            mesh.texCords, mesh.indices);
  REQUIRE(!adjList.empty( ));
  REQUIRE(adjList.size( ) ==
          mesh.indices.size( ) * 2);// Since the matrix holds NxN number of
                                    // values max, then size should be 2 * N
  const loader::Vertex vertex{ mesh.positions[1], mesh.normals[1],
                               mesh.texCords[1] };
  //std::cout << mesh.positions.size( ) << std::endl;
  //std::cout << vertex << std::endl;
  auto it = adjList.find(vertex);
  while (it != adjList.end( ))
  {
    auto lambda = [vertex](auto item) {
      if (vertex == item.first)
      {
        std::cout << item.second << std::endl;
      }
    };
    //lambda(*it);
    ++it;
  }
  const auto size = adjList.count(vertex);
  for(const auto& item: adjList)
  {
//    REQUIRE(adjList.count(item.first) == size);
  }
  REQUIRE(size == 4);
}
