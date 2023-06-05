#pragma once
#include <GLFW/glfw3.h>
#include <math.h>
#include <Matrix.h>
#include <Graphics.h>
using namespace std;

#ifndef PHYSICS_H
#define PHYSICS_H

double deltaTime = 0;
int fps = 0;
void Time()
{
    static double prevTime = 0;
    double currentTime = glfwGetTime();
    deltaTime = currentTime - prevTime;
    prevTime = currentTime;

    static double t = 0;
    static int frames = 0;
    
    t += deltaTime;
    frames++;
    fps = ((double)frames) / t;
    std::cout << "FPS:" << fps << std::endl;
    if (t >= 1.0)
    {
        t = 0;
        frames = 0;
    }
}

const float accel = 0.1;
const float deccel = 0.95;
const float defaultMoveSpeed = 50;
float moveSpeed = defaultMoveSpeed;
float rotateSpeed = PI / 2;
bool isKinematic = false;
bool dampenersActive = true;
Vec3 moveDir = Vec3(0, 0, 0);
Vec3 velocity = Vec3(0, 0, 0);

static void Physics(GLFWwindow* window)
{
    //---------- Physics Update ------------
    if (!isKinematic) {
        velocity += moveDir * moveSpeed * accel;
        if (dampenersActive) {
            velocity *= 0.95;
        }
        Camera::main->position += velocity * deltaTime;
        //std::cout << "Velocity:" << velocity.ToString();
    }
    else {
        Camera::main->position += moveDir * moveSpeed * deltaTime;
    }

    Mesh::meshes[0]->rotation = Matrix3x3::RotZ(2 * PI * deltaTime) * Mesh::meshes[0]->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);

    for (size_t i = 0; i < Mesh::meshes.size(); i++)
    {
        if (Mesh::meshes[i])
        {
            //Mesh::meshes[i]->position += (World::up * deltaTime);
        }
    }
}
#endif