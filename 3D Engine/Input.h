#pragma once
#ifndef INPUT_H
#define INPUT_H
#include <GLFW/glfw3.h>
#include <Graphics.h>
#include <Physics.h>
//-----------------Input----------------------

//static double prevMouseX;
//static double prevMouseY;
static double deltaMouseX;
static double deltaMouseY;
float mouseSensitivity = .25;
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

        double xAngle = mouseSensitivity * -deltaMouseY * deltaTime;
        double yAngle = 0.0000001 + mouseSensitivity * -deltaMouseX * deltaTime; // 0.0000001 so no gimbal lock
        Camera::main->rotation *= YPR(xAngle, yAngle, 0);
        //Camera::main->rotation = Matrix3x3::RotX(rotateSpeed * -deltaMouseY) * Camera::main->rotation * Matrix3x3::RotY((0.00001 + rotateSpeed) * -deltaMouseX);
    }
}

void OnScrollEvent(GLFWwindow* window, double xOffset, double yOffset)
{
    FOV(fieldOfViewDeg - yOffset);

    std::cout << "FOV:" << ToDeg(fov) << "°" << std::endl;
}

void OnMouseButtonEvent(GLFWwindow* window, int button, int action, int mods)
{
    std::cout << "Mouse button:" << button << std::endl;

    if (action == GLFW_PRESS)
    {
        // Spawn Mesh
        if (button == 1) {
            Mesh* mesh = LoadMeshFromOBJFile("Objects/Sphere.obj");
            mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
        }
    }
}

void OnKeyPressEvent(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS)
    {
        // Reset Camera
        if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS) {
            // Camera::main->reset();
            //Fix later
            Camera::main->rotation = Identity3x3;
            Camera::main->position = Vec3();
        }
        else if (glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS) {
            Camera::main = Camera::cameras[0];
        }
        else if (glfwGetKey(window, GLFW_KEY_F2) == GLFW_PRESS) {
            Camera::main = Camera::cameras[1];
        }

        //------------------Physics-------------------

        // Toggle momentum
        else if (key == GLFW_KEY_X) {
            velocity = Vec3(0, 0, 0);//Reset every toggle state
            isKinematic = !isKinematic;
        }

        // Toggle dampeners
        else if (key == GLFW_KEY_Z) {
            dampenersActive = !dampenersActive;
        }

        //-------------------Debugging------------------------

        else if (key == GLFW_KEY_I) {
            GraphicSettings::invertNormals = !GraphicSettings::invertNormals;
        }
        else if (key == GLFW_KEY_N) {
            GraphicSettings::debugNormals = !GraphicSettings::debugNormals;
        }
        else if (key == GLFW_KEY_V) {
            GraphicSettings::culling = !GraphicSettings::culling;
        }
        else if (key == GLFW_KEY_P) {
            GraphicSettings::perspective = !GraphicSettings::perspective;
        }
        else if (key == GLFW_KEY_M) {
            GraphicSettings::fillTriangles = !GraphicSettings::fillTriangles;
        }
        else if (key == GLFW_KEY_3) {
            GraphicSettings::displayWireFrames = !GraphicSettings::displayWireFrames;
        }
        else if (key == GLFW_KEY_L) {
            GraphicSettings::lighting = !GraphicSettings::lighting;
        }
        else if (key == GLFW_KEY_ESCAPE)
        {
            mouseCameraControlEnabled = !mouseCameraControlEnabled;
        }
        else if (key == GLFW_KEY_SLASH)
        {
            GraphicSettings::vfx = !GraphicSettings::vfx;
        }
    }
}

static void Input(GLFWwindow* window)
{
    //CheckMouseMove(window);

    moveDir = Vec3(0, 0, 0);
    //----------Camera Controls-------
    // FORWARD
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        moveDir += Camera::main->Forward();
    }
    // BACK
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        moveDir += Camera::main->Back();
    }
    // LEFT
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        moveDir += Camera::main->Left();
    }
    // RIGHT
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        moveDir += Camera::main->Right();
    }
    // UP
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        moveDir += Camera::main->Up();
    }
    // DOWN
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
        moveDir += Camera::main->Down();
    }

    moveDir.Normalize();

    // ROTATE CCW
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        Camera::main->rotation *= Matrix3x3::RotZ(rotateSpeed * deltaTime);
    }
    // ROTATE CW
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        Camera::main->rotation *= Matrix3x3::RotZ(-rotateSpeed * deltaTime);
    }
    // LOOK UP
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        Camera::main->rotation *= Matrix3x3::RotX(rotateSpeed * deltaTime);
    }
    // LOOK DOWN
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        Camera::main->rotation *= Matrix3x3::RotX(-rotateSpeed * deltaTime);
    }
    // TURN LEFT
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        Camera::main->rotation *= Matrix3x3::RotY(rotateSpeed * deltaTime);
    }
    // TURN RIGHT
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        Camera::main->rotation *= Matrix3x3::RotY(-rotateSpeed * deltaTime);
    }
    // Speed 
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    {
        moveSpeed = defaultMoveSpeed * 5;
    }
    else {
        moveSpeed = defaultMoveSpeed;
    }
}

#endif