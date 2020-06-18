//
// Created by hadalhw17 on 6/15/20.
//
#include "mesh_tools.hpp"
#include "mesh_loader.hpp"
#include <fmt/format.h>
#include "config.hpp"

int main()
{
  auto model = loader::loadMesh(
      fmt::format("{}{}", APP_PATH, "/test/dragon.obj").c_str( ));
  auto &mesh = model.value( ).meshes[0];
  const auto sdf = loader::generateSignedDistanceFieldFromMesh(mesh, 50);
  loader::saveSdfAsPPMA(sdf, fmt::format("{}{}", APP_PATH, "/profile/sdf.ppm").c_str());
}
