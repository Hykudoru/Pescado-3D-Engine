#pragma once
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <Utility.h>
extern GLFWwindow* window;

void InitUI()
{
    // IMGUI
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.Fonts->AddFontDefault();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 460");
    ImGui::StyleColorsDark();
}

void UI()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (mouseCameraControlEnabled)
    {
        ImGui::SetMouseCursor(ImGuiMouseCursor_None);
    }

    ImGui::Begin("Debugger");

    for (auto text : debugLog)
    {
        ImGui::Text(text.c_str());
    }
    for (auto text : debugLog2)
    {
        ImGui::Text(text.c_str());
    }

    ImGui::End();
    

    ClearLog();
    ClearLog2();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}