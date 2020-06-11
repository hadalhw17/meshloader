# cpp_base

[![Build Status](https://travis-ci.com/hadalhw17/cpp_base.svg?branch=master)](https://travis-ci.com/hadalhw17/cpp_base)

Ultimate C++ starter project template with CMake for graphics/game programming

Uses conan to get such modules as glfw, spdlog, fmt, glm, glad and docopt.
For ImGui it uses CMake to add a submodule if you need it
The reason why conan wasn't used for ImGui is because conan package does not contain imgui/examples directory, which is essential if you want to use your custom rendering backends

**Install**
1. Install Conan with **pip** ```pip install conan```
2. Create your repository with ```git init```
3. Pull the history of this repository using ```git pull http://github.com/hadalhw17/cpp_base.git```
4. Create build directory ```mkdir build && cd build```
5. Configure your cmake file with ```cmake ..``` or ```ccmake ..```
6. You are wonderful **<3**
