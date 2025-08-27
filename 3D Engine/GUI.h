#pragma once
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "misc/cpp/imgui_stdlib.h"
#include <cstring>
#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
#include "Utility.h"
#include "Input.h"
using namespace std;
namespace fs = std::filesystem;
extern std::function<PhysicsObject* ()> spawn;
static List<const char*> assetFiles = {};
static List<string> files;
static List<string> filesCopy;
void LoadAssets()
{
    files.clear();
    filesCopy.clear();
    assetFiles.clear();
    assetFiles.emplace_back("Cube");
    fs::path path("./Objects");
    for (const auto& file : fs::directory_iterator(path))
    {
        string fullFileName = file.path().filename().string();
        string fileExt = fullFileName.substr(fullFileName.length() - 3, 3);
        if (fileExt == "obj") {
            files.emplace_back(fullFileName);
            filesCopy.emplace_back(fullFileName);
        }
    }
    for (const auto& name : files)
    {
        assetFiles.emplace_back(name.c_str());
    }
}

void RenameFile(string original, string newName)
{
    fs::path path("./Objects");
    fs::rename(path / original, path / newName);
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

void DebuggerWindow()
{
    static bool foo = true;
    ImGui::Begin("Debugger", &foo, ImGuiWindowFlags_NoBackground);
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
        ImGui::Text("Switch Camera (Press F2)");
        ImGui::Text("Grab Object (Left Mouse Button)");
        ImGui::Text("Spawn Object (Right Mouse Button)");
        ImGui::Text("Delete Object (Aim and press Delete key)");

        ImGui::Text("\nFast-Forward Time (Hold PageUP)");
        ImGui::Text("Reverse Time (Hold PageDown)");
    }
    ImGui::End();
}

void ControlsWindow()
{
    ImGui::Begin("Global Settings");
    {
        ImGui::SeparatorText("CAMERA");
        ImGui::Text(("Position: (" + to_string(Camera::main->Position().x) + ", " + to_string(Camera::main->Position().y) + ", " + to_string(Camera::main->Position().z) + ")").c_str());
        ImGui::Text(("Velocity: <" + to_string(velocity.x) + ", " + to_string(velocity.y) + ", " + to_string(velocity.z) + ">").c_str());
        static float val = fieldOfViewDeg;
        if (ImGui::SliderFloat("FOV", &val, 1, 179))
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
        ImGui::Checkbox("View Wireframe", &Graphics::displayWireFrames);
        bool ignore = !Graphics::lighting;
        ImGui::Checkbox("Ignore Lighting", &ignore);
        Graphics::lighting = !ignore;
        ImGui::Checkbox("View Normals", &Graphics::debugNormals);
        ImGui::Checkbox("Invert Normals", &Graphics::invertNormals);
        ToolTip("This will flip triangle faces making meshes appear inside out or visible within.");
        ImGui::Checkbox("View Axes", &Graphics::debugAxes);
        ImGui::Checkbox("View Bounding Box", &Graphics::debugBounds);
        ImGui::Checkbox("View OctTree", &Graphics::debugTree);
        ImGui::Checkbox("Backface Culling", &Graphics::backFaceCulling);
        ToolTip("Backface culling will ignore any triangles not facing the camera. \nDisable this while in x-ray mode to see both the front and back of wireframes.");
        bool disable = !Graphics::frustumCulling;
        ImGui::Checkbox("Disable Frustum Culling", &disable);
        Graphics::frustumCulling = !disable;
        ToolTip("Not recommended. Fun to experiment with though if you're learning how a graphics engine works.");

        ImGui::SeparatorText("PHYSICS");
        ImGui::Checkbox("Time", &Physics::time);
        ImGui::Checkbox("Gravity", &Physics::gravity);
        ImGui::Checkbox("Simulate Physics", &Physics::dynamics);
        ToolTip("When disabled, objects will not be affected by gravity, collision impulses, or acceleration.");
        ImGui::Checkbox("Collision Detection", &Physics::collisionDetection);
        ToolTip("When disabled, collisions will be ignored.");
        ImGui::Checkbox("OctTree Collisions", &Physics::octTree);
        ToolTip("When enabled, may result in better CPU performance if there are too many colliders.");
    }
    ImGui::End();
}

void AssetWindow()
{
    ImGui::Begin("Assets");
    ImGui::SameLine();
    ToolTip("Double click an object to spawn it or just select one then use the right mouse button while in the world.\n- Assets are located inside: 3D Engine/Objects/");
    {
        static int selected = -1;
        ImGui::BeginChild("Assets");
        for (int i = 0; i < assetFiles.size(); i++)
        {
            auto fileName = assetFiles[i];
            if (ImGui::Selectable(fileName, selected == i, ImGuiSelectableFlags_AllowDoubleClick))
            {
                selected = i;
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                {
                    auto obj = spawn();
                    obj->localPosition = Camera::main->Position() + (Camera::main->Forward() * 8 * obj->mesh->bounds->max.Magnitude() * 0.5);
                    obj->localRotation = Camera::main->Rotation();
                }
            }
            if (i != 0)
            {
                if (ImGui::BeginPopupContextItem())
                {
                    selected = i;
                    static char buff[64];
                    ImGui::InputTextWithHint("", fileName, buff, 64);
                    if (ImGui::Button("Save") || ImGui::IsKeyDown(ImGuiKey_Enter))
                    {
                        ImGui::CloseCurrentPopup();
                        if (buff[0] != ' ') {
                            RenameFile(assetFiles[i], buff);
                            LoadAssets();
                        }
                        buff[0] = '\0';
                    }
                    if (ImGui::Button("Close"))
                    {
                        ImGui::CloseCurrentPopup();
                        buff[0] = '\0';
                    }
                    ImGui::EndPopup();
                }
            }
        }
        ImGui::EndChild();

        if (selected >= 0)
        {
            string objName = assetFiles[selected];
            if (objName == "Cube")
            {
                spawn = []() { return new PhysicsObject(new CubeMesh(), new BoxCollider()); };
            }
            else if (objName == "Sphere")
            {
                spawn = []() { return new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider()); };
            }
            else {
                spawn = [objName]() { return new PhysicsObject(LoadMeshFromOBJFile(objName), new BoxCollider()); };
            }
        }
    }
    ImGui::End();
}

void Inspector()
{
    ImGui::Begin("Inspector");
    if (prevGrabInfo.objectHit)
    {
        auto mesh = prevGrabInfo.objectHit;
        auto phys = prevGrabInfo.objectHit->object;
        {
            ImGui::Text(("Position: (" + to_string(mesh->Position().x) + ", " + to_string(mesh->Position().y) + ", " + to_string(mesh->Position().z) + ")").c_str());
            if (phys) 
            {
                ImGui::Text(("Velocity: <" + to_string(phys->velocity.x) + ", " + to_string(phys->velocity.y) + ", " + to_string(phys->velocity.z) + ">").c_str());
            }
            ImGui::SeparatorText("GRAPHICS");
            ImGui::Checkbox("View Wireframe", &mesh->forceWireFrame);
            ImGui::Checkbox("Ignore Lighting", &mesh->ignoreLighting);

            ImGui::SeparatorText("PHYSICS");
            if (phys)
            {
                ImGui::Checkbox("Is Kinematic", &phys->isKinematic);
                ToolTip("When enabled, objects will not be affected by gravity, impulses, or acceleration.");
                bool isTrigger = phys->IsTrigger();
                ImGui::Checkbox("Is Trigger", &isTrigger);
                phys->IsTrigger(isTrigger);
                ToolTip("When enabled, collisions will be ignored.");
            }
        }
    }
    ImGui::End();
}

void InitGUI()
{
    // IMGUI
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->AddFontDefault();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 460");
    ImGui::StyleColorsDark();

    LoadAssets();
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
    if (!ImGui::IsWindowHovered(ImGuiFocusedFlags_AnyWindow) 
        && !ImGui::IsAnyItemHovered()
        && !ImGui::IsWindowFocused(ImGuiFocusedFlags_AnyWindow)
        && ImGui::IsAnyMouseDown()) {
        mouseCameraControlEnabled = true;
    }

    DebuggerWindow();
    ControlsWindow();
    AssetWindow();
    Inspector();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}