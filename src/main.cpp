#include <spdlog/spdlog.h>
#include <fmt/printf.h>
#include "imgui.h"
#include "examples/imgui_impl_glfw.h"
#include "examples/imgui_impl_opengl3.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <cassert>


int main()
{
    if(!glfwInit())
    {
        return 1;
    }
    GLFWwindow *window = glfwCreateWindow(640, 480, "Demo Window", nullptr, nullptr);
    if(!window)
    {
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    if(!gladLoadGL())
    {
        return 1;
    }

    ImGui::CreateContext();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    spdlog::info("Hello {}!", "World");
    fmt::print("Hello {} from fmt", "World!");

    while(!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        //io.DisplaySize = ImVec2(640, 480);
        //io.DeltaTime = 1.f / 60.f;
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        bool show = true;
        ImGui::ShowDemoWindow(&show);
        ImGui::Render();
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);

    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
