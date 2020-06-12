#pragma once
#include "structures.hpp"

#include <optional>

namespace loader
{
[[nodiscard]] std::optional<Model> loadGlTf(const char *InPath);
}
