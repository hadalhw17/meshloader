// meshloader.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include "mesh_loader.hpp"

#include "obj_loader.hpp"
#include "gltf_loader.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <vector>

#define TEST_OBJ 1
#define TEST_GLTF 0

namespace loader
{
std::optional<Model> loadMesh(const char *InFilename)
{
  const std::filesystem::path path(InFilename);
  if (path.extension( ) == ".obj")
  {
    return CObjModel::LoadObjFromFile(InFilename);
  }
  if (path.extension( ) == ".gltf")
  {
    return loadGlTf(InFilename);
  }
  return std::nullopt;
}
}
#ifdef TEST
int main( )
{
  using namespace std::chrono;
#if TEST_OBJ
  auto begin = high_resolution_clock::now( );
  auto objModel = loader::loadMesh("/home/hadalhw17/dev/meshloader/test/dragon.obj");
  auto end = high_resolution_clock::now( );
  duration<double> time = duration_cast<duration<double>>(end - begin);
  std::cout << "Obj model loaded in " << time.count( ) << " seconds!"
            << std::endl;
#endif

#if TEST_GLTF
  auto begin = high_resolution_clock::now( );
  auto gltfModel =
      loader::loadMesh("/home/hadalhw17/dev/meshloader/test/cube_gltf/Cube.gltf");
  auto end = high_resolution_clock::now( );

  auto time = duration_cast<duration<double>>(end - begin);
  std::cout << "GLTF model loaded in " << time.count( ) << " seconds!"
            << std::endl;
#endif

  return 0;
}
#endif
