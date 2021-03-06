cmake_minimum_required(VERSION 3.15)

# Set the project name to your project name, my project isn't very descriptive
project(meshloader CXX)
include(cmake/ProjectSettings.cmake)


option(USE_CLANG "Use clang toolchain" OFF)
if(USE_CLANG)
    include(cmake/ClangToolchain.cmake)
endif()

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY CXX_STANDARD 20)
target_compile_features(project_options INTERFACE cxx_std_20)

#set_target_properties(
#    PROPERTIES
#    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIR})
if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    option(ENABLE_BUILD_WITH_TIME_TRACE "Enable -ftime-trace to generate time tracing .json files on clang" OFF)
    if (ENABLE_BUILD_WITH_TIME_TRACE)
        add_compile_definitions(project_options INTERFACE -ftime-trace)
    endif()
endif()

option(WARNINGS_AS_ERRORS "Treat all warnigns as errors" FALSE)
# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

# enable cache system
include(cmake/Cache.cmake)

# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

# sanitizer options if supported by compiler
include(cmake/Sanitizers.cmake)
enable_sanitizers(project_options)

# enable doxygen
include(cmake/Doxygen.cmake)
enable_doxygen()

# allow for static analysis options
include(cmake/StaticAnalyzers.cmake)

option(BUILD_SHARED_LIBS "Enable compilation of shared libraries" OFF)
option(ENABLE_TESTING "Enable Test Builds" ON)
option(ENABLE_FUZZING "Enable Fuzzing Builds" OFF)

# Very basic PCH example
option(ENABLE_PCH "Enable Precompiled Headers" OFF)
if (ENABLE_PCH)
    # This sets a global PCH parameter, each project will build its own PCH, which
    # is a good idea if any #define's change
    #
    # consider breaking this out per project as necessary 
    target_precompile_headers(project_options INTERFACE <vector> <string> <map> <utility>)
endif()


# Set up some extra Conan dependencies based on our needs
# before loading Conan
set(CONAN_EXTRA_REQUIRES "")
set(CONAN_EXTRA_OPTIONS "")

if(USE_IMGUI)
    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/imgui")
        set(CLONE_CMD "git")
        set(CLONE_ARGS submodule add --force https://github.com/ocornut/imgui.git ${PROJECT_SOURCE_DIR}/extern/imgui)
        MESSAGE(Executing command: "${CLONE_CMD} ${CLONE_ARGS}")
        execute_process(
                COMMAND ${CLONE_CMD} ${CLONE_ARGS}
        )
    endif()
endif()

if(USE_GLM)
    set(CONAN_EXTRA_REQUIRES ${CONAN_EXTRA_REQIUIRES}
            glm/0.9.9.8)
endif()

if(USE_GLFW)
    set(CONAN_EXTRA_REQUIRES ${CONAN_EXTRA_REQUIRES}
        glfw/3.3.2@bincrafters/stable)
endif()

if(USE_GLAD)
    set(CONAN_EXTRA_REQUIRES ${CONAN_EXTRA_REQUIRES}
            glad/0.1.33)
endif()


include(cmake/Conan.cmake)
run_conan()

if(ENABLE_TESTING)
    enable_testing()
    message(
        "Building Tests. Be sure to check out test/constexpr_tests for constexpr testing"
        )
    add_subdirectory(test)
endif()

if(ENABLE_FUZZING)
    message(
        "Building Fuzz Tests, using fuzzing sanitizer https://www.llvm.org/docs/LibFuzzer.html"
        )
    add_subdirectory(fuzz_test)
endif()


add_subdirectory(src)
add_subdirectory(profile)


option(ENABLE_UNITY "Enable Unity builds of projects" OFF)
if (ENABLE_UNITY)
    # Add for any project you want to apply unity builds for
    set_target_properties(loader PROPERTIES UNITY_BUILD ON)
endif()

option(APP_PATH_DEF "Define application path in the config" ON)
if(APP_PATH_DEF)
    set(APP_PATH ${PROJECT_SOURCE_DIR})
endif()

include_directories(src)
configure_file(src/config.h.in ${PROJECT_SOURCE_DIR}/src/config.hpp @ONLY)