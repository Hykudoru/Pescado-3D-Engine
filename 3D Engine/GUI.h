#pragma once
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <Utility.h>
extern GLFWwindow* window;

void InitGUI()
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

void ToolTip(const char* description)
{
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) 
    {
        //ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(description);
        //ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void GUI()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (mouseCameraControlEnabled)
    {
        ImGui::SetMouseCursor(ImGuiMouseCursor_None);
    }

    ImGui::Begin("Debugger");
    {
        ImGui::SeparatorText("GRAPHICS");
        ImGui::Text(("FPS:" + to_string(fps)).c_str());
        ImGui::Text(("Frame Time:" + to_string(1.0 / (double)fps)).c_str());
        ImGui::Text(("Meshes:" + to_string(Mesh::count)).c_str());
        ImGui::Text(("Triangles Drawn:" + to_string(Mesh::worldTriangleDrawCount)).c_str());

        ImGui::SeparatorText("PHYSICS");
        ImGui::Text(("Colliders: " + to_string(Collider::count)).c_str());
        ImGui::Text(("Sphere Colliders: " + to_string(ManagedObjectPool<SphereCollider>::count)).c_str());
        ImGui::Text(("Box Colliders: " + to_string(ManagedObjectPool<BoxCollider>::count)).c_str());
        ImGui::Text(("Plane Colliders: " + to_string(ManagedObjectPool<PlaneCollider>::count)).c_str());

        ImGui::SeparatorText("CAMERA CONTROLS");
        ImGui::Text("Move Forward/Back/Left/Right (W,A,S,D)");
        ImGui::Text("Move Up (Spacebar)");
        ImGui::Text("Move Down (C)");
        ImGui::Text("Rotate (Q and E)");
        ImGui::Text("Sprint (Left Shift)");
        ImGui::Text("Insane Sprint (Right Ctrl)");
        ImGui::Text("Reset Camera (Press 0)");
        ImGui::Text("Grab (Right Mouse Button)");
        ImGui::Text("\nFast-Forward Time (Hold PageUP)");
        ImGui::Text("Reverse Time (Hold PageDown)");
    }
    ImGui::End();
    

    

    ImGui::Begin("Controls");
    {
        ImGui::SeparatorText("CAMERA");
        ImGui::Text(("Position: (" + to_string(Camera::main->Position().x) + ", " + to_string(Camera::main->Position().y) + ", " + to_string(Camera::main->Position().z) + ")").c_str());
        ImGui::Text(("Velocity: <" + to_string(velocity.x) + ", " + to_string(velocity.y) + ", " + to_string(velocity.z) + ">").c_str());
        static float val = fieldOfViewDeg;
        if (ImGui::SliderFloat("FOV", &val, 30, 90))
        {
            FOV(val);
        }
        ImGui::Checkbox("Kinematic (Press X)", &isKinematic);
        ToolTip("Gives the camera unrealistic instantaneous movement.");
        ImGui::Checkbox("Inertial Dampeners (Press Z)", &dampenersActive);
        ToolTip("When Kinematic is disabled, the camera will slowly come to a stop over time.");
        

        ImGui::SeparatorText("GRAPHICS");
        static string mode = "Enable X-Ray Mode";
        if (ImGui::Button(mode.c_str()))
        {
            Graphics::fillTriangles = !Graphics::fillTriangles;
            if (Graphics::fillTriangles)
            {
                mode = "Enable X-Ray Mode"; // actually means we are currently in normal mode
                Graphics::displayWireFrames = false;
            }
            else
            {
                mode = "Enable Normal Mode"; // actually means we are currently in x-ray mode
            }
        }
        ImGui::Checkbox("Wireframe", &Graphics::displayWireFrames);
        ImGui::Checkbox("Enable Backface Culling", &Graphics::backFaceCulling);
        ToolTip("Backface culling will ignore any triangles not facing the camera. \nDisable this while in x-ray mode to see both the front and back of wireframes.");
        ImGui::Checkbox("Normals", &Graphics::debugNormals);
        ImGui::Checkbox("Invert Normals", &Graphics::invertNormals);
        ImGui::Checkbox("Axes", &Graphics::debugAxes);
        ImGui::Checkbox("Bounding Box", &Graphics::debugBounds);
        ImGui::Checkbox("OctTree", &Graphics::debugTree);


        ImGui::SeparatorText("PHYSICS");
        ImGui::Checkbox("Gravity", &Physics::gravity);
        ImGui::Checkbox("Rigidbody Physics", &Physics::dynamics);
        ToolTip("When disabled, objects will not be affected by gravity, collision impulses, or acceleration.");
        ImGui::Checkbox("Collision Detection", &Physics::collisionDetection);
        ToolTip("When disabled, collisions will be ignored.");
        ImGui::Checkbox("OctTree Collisions", &Physics::octTree);
        ToolTip("When enabled, may result in better CPU performance if there are too many colliders.");

    }
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}