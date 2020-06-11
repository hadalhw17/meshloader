option(USE_IMGUI "Add ImGui to the project as a submodule" OFF)
option(USE_GLFW "Clone GLFW to the project with conan" OFF)
option(USE_GLAD "Clone GLAD to the project with conan" OFF)
option(USE_VULKAN "Use Vulkan SDK as a rendering backend" OFF)
option(USE_GLM "Clone GLM to the project with conan" OFF)

set(CONAN_PACKAGES "")
set(EXTERNAL_INCL "")
set(EXTERNAL_SRC "")
set(INCLUDE_DIRS "")

if(USE_VULKAN)
    find_package(Vulkan REQUIRED)

    if(UNIX)
        if(Vulkan_FOUND)
            find_library(Vulkan_LIBRARY NAMES vulkan HINTS "$ENV{VULKAN_SDK}/lib" "${CMAKE_SOURCE_DIR}/libs/vulkan" REQUIRED)
            if (Vulkan_LIBRARY)
                set(Vulkan_FOUND ON)
                MESSAGE("Using bundled Vulkan library version")
            endif()
        endif()
    endif()
    IF (NOT Vulkan_FOUND)
        message(FATAL_ERROR "Could not find Vulkan library!")
    ELSE()
        message(STATUS ${Vulkan_LIBRARY})
    ENDIF()
    set(CONAN_PKG ${CONAN_PKG} ${Vulkan_LIBRARY} Vulkan::Vulkan)
    set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Vulkan_INCLUDE_DIR})
endif()

if(USE_IMGUI)
    set(IMGUI_SRC "")
    set(IMGUI_INCL "")
    set(IMGUI_DIR ${PROJECT_SOURCE_DIR}/extern/imgui)
    # IMGUI common source files
    set(IMGUI_SRC ${IMGUI_SRC}
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_demo.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp)

    # IMGUI backend specific sources
    if(USE_GLFW)
        set(IMGUI_SRC ${IMGUI_SRC}
            ${IMGUI_DIR}/examples/imgui_impl_glfw.cpp)
    endif()
    if(USE_GLAD)
        set(IMGUI_SRC ${IMGUI_SRC}
            ${IMGUI_DIR}/examples/imgui_impl_opengl3.cpp)
    endif()
    if(USE_VULKAN)
        set(IMGUI_SRC ${IMGUI_SRC}
            ${IMGUI_DIR}/examples/imgui_impl_vulkan.cpp)
    endif()
    set(IMGUI_INCL ${INCLUDE_DIRS}
        ${IMGUI_DIR}/
        ${IMGUI_DIR}/examples)

    add_library(ImGui STATIC ${IMGUI_SRC})
    target_include_directories(ImGui SYSTEM PUBLIC ${IMGUI_INCL})
    target_link_libraries(ImGui PUBLIC project_options)
    set(CONAN_PACKAGES ${CONAN_PACKAGES} ImGui)
endif()

if(USE_GLFW)
    find_package(glfw3 CONFIG REQUIRED)
    set(CONAN_PACKAGES ${CONAN_PACKAGES} CONAN_PKG::glfw)
endif()

if(USE_GLAD)
    set(CONAN_PACKAGES ${CONAN_PACKAGES} CONAN_PKG::glad)
endif()

if(USE_GLM)
    set(CONAN_PACKAGES ${CONAN_PACKAGES} CONAN_PKG::glm)
endif()

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()
#find_package(spdlog CONFIG REQUIRED)
#find_package(fmt CONFIG REQUIRED)

add_executable(intro main.cpp ${EXTERNAL_SRC} )
include_directories(intro PRIVATE ${INCLUDE_DIRS})
target_link_libraries(intro PRIVATE project_options project_warnings CONAN_PKG::docopt.cpp
        CONAN_PKG::fmt CONAN_PKG::spdlog ${CONAN_PACKAGES})

if(USE_VULKAN)
    target_link_libraries(intro PRIVATE Vulkan::Vulkan)
endif()
