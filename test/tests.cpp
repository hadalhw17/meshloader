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

TEST_CASE("Test converted if .gltf cube generated from .obj cube is loaded in "
          "the same way",
          "[cube_validator]")
{
  auto model = loader::loadMesh(
      fmt::format("{}{}", APP_PATH, "/test/cube.obj").c_str( ));
  auto model1 = loader::loadMesh(
      fmt::format("{}{}", APP_PATH, "/test/cube.gltf").c_str( ));

  auto &mesh = model.value().meshes[0];
  auto &mesh1 = model1.value().meshes[0];

  auto lambda = [&](auto case1, auto case2)
  {
    REQUIRE(case1.size() == case2.size());
    for(size_t i = 0; i < case1.size(); ++i)
    {
      REQUIRE(case1[i] == case2[i]);
    }
  };
  lambda(mesh.indices, mesh1.indices);
  lambda(mesh.positions, mesh1.positions);
  //lambda(mesh.normals, mesh1.normals);
  //lambda(mesh.texCords, mesh1.texCords);
}

TEST_CASE("Testing vertex adjacency list", "[vert_adj]")
{
  auto model = loader::loadMesh(
      fmt::format("{}{}", APP_PATH, "/test/cube.obj").c_str( ));
  auto &mesh = model.value( ).meshes[0];
  auto adjList = loader::getVertexAdjacency(mesh.positions, mesh.normals,
                                            mesh.texCords, mesh.indices);
  REQUIRE(!adjList.empty( ));
  REQUIRE(adjList.size( ) == mesh.positions.size( ));
  const loader::Vertex vertex{
    mesh.positions[1], mesh.normals[1],
    mesh.texCords.empty( ) ? loader::float2{ 0.f, 0.f} : mesh.texCords[1]
  };// This vertex should have 4 neighbours as it is on the adjacent edge

  const loader::Vertex vertex1{
    mesh.positions[2], mesh.normals[2],
    mesh.texCords.empty( ) ? loader::float2{ 0.f, 0.f} : mesh.texCords[2]
  };// This vertex should have 2 adjacent vertices as it is a 90 degree corner
    // of a triangle

  for (const auto &item : adjList)
  {
    REQUIRE(((item.second.size( ) == 2) ||
             (item.second.size( ) ==
              4)));// Cube should have 2 or 4 adjacent
                   // vertices per vertex (2 for corners and 4 for vertices that
                   // share the edge between the 2 triangles of a face)
  }

  REQUIRE(adjList[vertex].size( ) == 4);
  REQUIRE(adjList[vertex1].size( ) == 2);
}
