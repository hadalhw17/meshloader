#include <fmt/printf.h>
#include <spdlog/spdlog.h>

struct TestStruct
{
  float a;
  float b;
};

int main( )
{
  spdlog::info("Hello {}!", "World");
  fmt::print("Hello {} from fmt", "World!");
  TestStruct s{ };
  TestStruct *structPointer;

  return s.a;
}
