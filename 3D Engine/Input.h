#pragma once
#ifndef INPUT_H
#define INPUT_H
#include <GLFW/glfw3.h>
#include <Graphics.h>
#include <Physics.h>
//-----------------Input----------------------
static double deltaMouseX;
static double deltaMouseY;
float mouseSensitivity = .1;
bool mouseCameraControlEnabled = true;

void OnMouseMoveEvent(GLFWwindow* window, double mouseX, double mouseY)
{
    if (mouseCameraControlEnabled)
    {
        static  double screenCenterX = (screenWidth / 2.0);
        static  double screenCenterY = (screenHeight / 2.0);

        glfwGetCursorPos(window, &mouseX, &mouseY);

        // Subtract centerX and centerY since the mouse will be forced to move back to the center of the screen before the next frame.
        deltaMouseX = mouseX - screenCenterX;
        deltaMouseY = mouseY - screenCenterY;

        // Reset cursor to the center of the screen
        glfwSetCursorPos(window, screenCenterX, screenCenterY);
        static float rad = PI / 180;
        double xAngle = rad * mouseSensitivity * -deltaMouseY;// *deltaTime;
        double yAngle = 0.0000001 + rad * mouseSensitivity * -deltaMouseX;// * deltaTime; // 0.0000001 so no gimbal lock
        if (CameraSettings::outsiderViewPerspective) {
            Camera::projector->rotation *= YPR(xAngle, yAngle, 0);
        }
        else {
            Camera::main->rotation *= YPR(xAngle, yAngle, 0);
        }
        //Camera::main->rotation = Matrix3x3::RotX(rotateSpeed * -deltaMouseY) * Camera::main->rotation * Matrix3x3::RotY((0.00001 + rotateSpeed) * -deltaMouseX);
    }
}

void OnMouseButtonEvent(GLFWwindow* window, int button, int action, int mods)
{
    std::cout << "Mouse button:" << button << std::endl;

    if (action == GLFW_PRESS)
    {
        // Spawn Mesh
        if (button == 0) {
            Mesh* mesh = new CubeMesh();//LoadMeshFromOBJFile("Objects/Sphere.obj");
            mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
            mesh->rotation = Camera::main->rotation;
            mesh->color = RGB::turquoise;
        }
        else if (button == 1) {
            Physics::raycasting = true;
        }
    }
    if (action == GLFW_RELEASE)
    {
        if (button == 1) {
            Physics::raycasting = false;
        }
    }
}

void OnScrollEvent(GLFWwindow* window, double xOffset, double yOffset)
{
    FOV(fieldOfViewDeg - yOffset);

    std::cout << "FOV:" << ToDeg(fov) << "�" << std::endl;
}

void OnKeyPressEvent(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS)
    {
        // Reset Camera
        if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS) {
            Camera::main->rotation = Identity3x3;
            Camera::main->position = Vec3();
        }
        // Switch between Cameras
        else if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) {
            Camera::main = Camera::cameras[1];
        }
        else if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) {
            Camera::main = Camera::cameras[2];
        }
        else if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS) {
            CameraSettings::outsiderViewPerspective = !CameraSettings::outsiderViewPerspective;
            Camera::projector->rotation = Identity3x3;
            Camera::projector->position = Vec3();
        }
        // Spawn
        else if (glfwGetKey(window, GLFW_KEY_9) == GLFW_PRESS) {
            Mesh* mesh = new CubeMesh();//LoadMeshFromOBJFile("Objects/Sphere.obj");
            mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
            mesh->rotation = Camera::main->rotation;
        }
        else if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS) {
            Mesh* mesh = LoadMeshFromOBJFile("Sphere.obj");
            mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
            mesh->rotation = Camera::main->rotation;
        }
        else if (glfwGetKey(window, GLFW_KEY_7) == GLFW_PRESS) {
            Mesh* mesh = LoadMeshFromOBJFile("Diamond.obj");
            mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
            mesh->rotation = Camera::main->rotation;
            mesh->scale *= 0.1;
            mesh->color = RGB::red;
        }
        else if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS) {
            Mesh* mesh = LoadMeshFromOBJFile("Icosahedron.obj");
            mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
            mesh->rotation = Camera::main->rotation;
            mesh->scale *= 0.1;
            mesh->color = RGB::purple;
        }

        //------------------Physics-------------------

        // Toggle Momentum
        else if (key == GLFW_KEY_X) {
            velocity = Vec3(0, 0, 0);//Reset every toggle state
            isKinematic = !isKinematic;
        }

        // Toggle Inertial Dampeners
        else if (key == GLFW_KEY_Z) {
            dampenersActive = !dampenersActive;
        }

        // Toggle Collision Detection
        else if (key == GLFW_KEY_P) {
            Physics::collisionDetection = !Physics::collisionDetection;
        }

        // Toggle Gravity
        else if (key == GLFW_KEY_G) {
            Physics::gravity = !Physics::gravity;
        }

        //-------------------Debugging------------------------
        else if (key == GLFW_KEY_I) {
            GraphicSettings::invertNormals = !GraphicSettings::invertNormals;
        }
        else if (key == GLFW_KEY_N) {
            GraphicSettings::debugNormals = !GraphicSettings::debugNormals;
        }
        else if (key == GLFW_KEY_V) {
            GraphicSettings::backFaceCulling = !GraphicSettings::backFaceCulling;
        }
        else if (key == GLFW_KEY_F) {
            GraphicSettings::fillTriangles = !GraphicSettings::fillTriangles;
        }
        else if (key == GLFW_KEY_M) {
            GraphicSettings::displayWireFrames = !GraphicSettings::displayWireFrames;
        }
        else if (key == GLFW_KEY_COMMA) {
            GraphicSettings::debugAxes = !GraphicSettings::debugAxes;
        }
        else if (key == GLFW_KEY_L) {
            GraphicSettings::lighting = !GraphicSettings::lighting;
        }
        else if (key == GLFW_KEY_ESCAPE)
        {
            mouseCameraControlEnabled = !mouseCameraControlEnabled;
        }
        else if (key == GLFW_KEY_F1)
        {
            GraphicSettings::vfx = !GraphicSettings::vfx;
        }
        else if (key == GLFW_KEY_F2)
        {
            GraphicSettings::matrixMode = !GraphicSettings::matrixMode;
        }
        // Toggle Transform Hierarchy
        else if (glfwGetKey(window, GLFW_KEY_CAPS_LOCK) == GLFW_PRESS) {
            Transform::parentHierarchyDefault = !Transform::parentHierarchyDefault;
        }
    }
}

extern GLFWwindow* window;
static void CameraControl(Camera* cam)
{
    moveDir = Vec3(0, 0, 0);
    //----------Camera Controls-------
    // FORWARD
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        moveDir += cam->Forward();
    }
    // BACK
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        moveDir += cam->Back();
    }
    // LEFT
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        moveDir += cam->Left();
    }
    // RIGHT
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        moveDir += cam->Right();
    }
    // UP
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        moveDir += cam->Up();
    }
    // DOWN
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
        moveDir += cam->Down();
    }

    moveDir.Normalize();

    // ROTATE CCW
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        cam->rotation *= Matrix3x3::RotZ(rotateSpeed * deltaTime);
    }
    // ROTATE CW
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        cam->rotation *= Matrix3x3::RotZ(-rotateSpeed * deltaTime);
    }
    // LOOK UP
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        cam->rotation *= Matrix3x3::RotX(rotateSpeed * deltaTime);
    }
    // LOOK DOWN
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        cam->rotation *= Matrix3x3::RotX(-rotateSpeed * deltaTime);
    }
    // TURN LEFT
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        cam->rotation *= Matrix3x3::RotY(rotateSpeed * deltaTime);
    }
    // TURN RIGHT
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        cam->rotation *= Matrix3x3::RotY(-rotateSpeed * deltaTime);
    }
    // Speed 
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    {
        accel = defaultAcceleration * 5;
    }
    else {
        accel = defaultAcceleration;
    }
}

static void Input()
{
    if (CameraSettings::outsiderViewPerspective)
    {
        CameraControl(Camera::projector);
    }
    else 
    {
        CameraControl(Camera::main);
    }
}

#endif