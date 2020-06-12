#pragma once
#include "structures.hpp"

#include <optional>

namespace loader
{
std::optional<Model> loadMesh(const char*InFilename);
}
