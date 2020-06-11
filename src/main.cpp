#include <spdlog/spdlog.h>
#include <fmt/printf.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <vulkan/vulkan.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <iostream>
#include <cassert>


int main()
{
    VkApplicationInfo appInfo = {};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.apiVersion = VK_API_VERSION_1_1;
    appInfo.applicationVersion = 1u;
    appInfo.engineVersion = 1u;
    appInfo.pApplicationName = "DemoApp";
    appInfo.pEngineName = "DemoEngine";

    VkInstanceCreateInfo instanceInfo = {};
    instanceInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    instanceInfo.pApplicationInfo = &appInfo;

    VkInstance instance;
    VkResult res = vkCreateInstance(&instanceInfo, nullptr, &instance);
    if(res != VK_SUCCESS)
    {
        spdlog::error("Vulkan failed to initialise(libraries not loaded)");
        return 1;
    }
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

    glm::vec3 test_a = glm::vec3(10.f, 0.f, 1.f);
    glm::vec3 test_b = glm::vec3(13.f, 1.f, 55.f);
    glm::vec3 res_glm = test_a + test_b;
    spdlog::info("glm test: (10.f, 0.f, 1.f)+(13.f, 1.f, 55.f) = ({}, {}, {})", res_glm.x, res_glm.y, res_glm.z);
    spdlog::info("Hello {}!", "World");
    fmt::print("Hello {} from fmt", "World!");
 
    bool show = true;
    while(!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        //io.DisplaySize = ImVec2(640, 480);
        //io.DeltaTime = 1.f / 60.f;
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
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
    vkDestroyInstance(instance, nullptr);
    return 0;
}
