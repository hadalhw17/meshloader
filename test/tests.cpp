#include "../src/mesh_loader.hpp"

#include <catch2/catch.hpp>
#include <optional>
#include "config.hpp"
#include "fmt/format.h"


TEST_CASE("Loading valid obj model", "[obj]")
{
  auto model =
      loader::loadMesh(
          fmt::format("{}{}", APP_PATH, "/test/dragon.obj").c_str());
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
      fmt::format("{}{}", APP_PATH, "/test/cube_gltf/Cube.gltf").c_str());
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
  auto model =
      loader::loadMesh("asdfewasdfesdf");
  REQUIRE(!model.has_value( ));
}

TEST_CASE("Loading invalid glTF model", "[glTF]")
{
  auto model = loader::loadMesh(
      "asdajkgsdfnksjldfbnlekjnsd");
  REQUIRE(!model.has_value( ));
}
