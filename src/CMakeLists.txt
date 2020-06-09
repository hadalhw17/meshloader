option(USE_IMGUI "Clone imgui to the project with conan" OFF)
option(USE_GLFW "Clone GLFW to the project with conan" OFF)
option(USE_GLAD "Clone GLAD to the project with conan" OFF)

set(CONAN_PACKAGES "")
set(EXTERNAL_INCL "")
set(EXTERNAL_SRC "")
set(INCLUDE_DIRS "")

if(USE_IMGUI)
    if(USE_GLAD)
        set(EXTERNAL_SRC ${EXTERNAL_SRC} ${PROJECT_SOURCE_DIR}/extern/imgui/examples/imgui_impl_glfw.cpp
                                            ${PROJECT_SOURCE_DIR}/extern/imgui/examples/imgui_impl_opengl3.cpp)
    endif()
    set(CONAN_PACKAGES ${CONAN_PACKAGES} ImGui)
    set(EXTERNAL_SRC ${EXTERNAL_SRC} ${IMGUI_SRC})
    set(EXTERNAL_INCL ${EXTERNAL_INCL} ${IMGUI_INLUDES})
    set(INCLUDE_DIRS ${INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/extern/imgui)
endif()

if(USE_GLFW)
    find_package(glfw3 CONFIG REQUIRED)
    set(CONAN_PACKAGES ${CONAN_PACKAGES} CONAN_PKG::glfw)
endif()

if(USE_GLAD)
    set(CONAN_PACKAGES ${CONAN_PACKAGES} CONAN_PKG::glad)
endif()

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()
#find_package(spdlog CONFIG REQUIRED)
#find_package(fmt CONFIG REQUIRED)

add_executable(intro main.cpp ${EXTERNAL_SRC} )
include_directories(intro PRIVATE ${INCLUDE_DIRS})
target_link_libraries(intro PRIVATE project_options project_warnings CONAN_PKG::docopt.cpp
    CONAN_PKG::fmt CONAN_PKG::spdlog ${CONAN_PACKAGES})
