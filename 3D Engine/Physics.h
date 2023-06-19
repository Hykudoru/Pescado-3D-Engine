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
    if (t >= 1.0)
    {
        t = 0;
        frames = 0;
    }
}

class PhysicsObject
{
public:
    float mass = 1;
    bool isKinematic = false;
    Vec3 velocity = Vec3(0, 0, 0);
    Vec3 angularVelocity = Vec3(0, 0, 0);
    Vec3 position;
    Matrix3x3 rotation;
    Mesh* mesh;

    PhysicsObject(Mesh* mesh)
    {
        this->mesh = mesh;
        position = mesh->position;
        rotation = mesh->rotation;
    }

    virtual void Update()
    {
        if (mesh) {
            mesh->position = position;
            mesh->rotation = rotation;
        }
    }
};

const float accel = 0.1;
const float deccel = 0.95;
const float defaultMoveSpeed = 50;
float moveSpeed = defaultMoveSpeed;
float rotateSpeed = PI / 2;
bool isKinematic = false;
bool dampenersActive = true;
Vec3 moveDir = Vec3(0, 0, 0);
Vec3 velocity = Vec3(0, 0, 0);

extern Mesh* planet; 
PhysicsObject physicsCube1 = PhysicsObject(new CubeMesh());// PhysicsObject();
PhysicsObject physicsCube2 = PhysicsObject(new CubeMesh());// PhysicsObject();

struct Range
{
    float min;
    float max;

    Range(float minimum, float maximum)
    {
        min = minimum;
        max = maximum;
    }
};

Range ProjectVertsOntoAxis(List<Vec3>& verts, Vec3& axis)
{
    float min = 0;
    float max = 0;
    for (size_t k = 0; k < verts.size(); k++)
    {
        float dist = DotProduct(verts[k], axis);
        if (dist < min) {
            min = dist;
        }
        else if (dist > max) {
            max = dist;
        }
    }

    return Range(min, max);
}

bool Collision(CubeMesh& mesh1, CubeMesh& mesh2)
{
    bool gap = false;
    if (!mesh1.vertices || !mesh2.vertices)
    {
        return false;
    }
    List<Vec3> mesh1Verts = *mesh1.vertices;
    List<Vec3> mesh1Normals = mesh1.WorldXYZNormals();

    List<Vec3> mesh2Verts = *mesh2.vertices;
    List<Vec3> mesh2Normals = mesh2.WorldXYZNormals();

    for (size_t i = 0; i < mesh1Verts.size(); i++)
    {
        mesh1Verts[i] = (Vec3)(mesh1.TRS() * (Vec4)mesh1Verts[i]);
    }
    for (size_t i = 0; i < mesh2Verts.size(); i++)
    {
        mesh2Verts[i] = mesh2.TRS() * (Vec4)mesh2Verts[i];
    }

    for (size_t i = 0; i < 2; i++)
    {
        for (size_t j = 0; j < mesh1Normals.size(); j++)
        {
            Vec3 n1 = mesh1Normals[j];

            Range mesh1Range = ProjectVertsOntoAxis(mesh1Verts, n1);
            Range mesh2Range = ProjectVertsOntoAxis(mesh2Verts, n1);
            gap = (!(mesh1Range.max > mesh2Range.min && mesh2Range.max > mesh1Range.min));
            if (gap) {
                break;
            }
        }
        if (gap) {
            break;
        }
        //Swap and repeat once more for other mesh
        mesh1Verts.swap(mesh2Verts);
        mesh1Normals.swap(mesh2Normals);
    }

    if (!gap) // must continue searching for possible 3D Edge-Edge collision
    {
        for (size_t i = 0; i < 3; i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                Vec3 axis = CrossProduct(mesh1Normals[i], mesh2Normals[j]);
                Range mesh1Range = ProjectVertsOntoAxis(mesh1Verts, axis);
                Range mesh2Range = ProjectVertsOntoAxis(mesh2Verts, axis);
                gap = !(mesh1Range.max > mesh2Range.min && mesh2Range.max > mesh1Range.min);
                if (gap) {
                    break;
                }
            }
            if (gap) {
                break;
            }
        }
    }

    return !gap;//No gap = collision
}

void DetectCollisions()
{
    CubeMesh* mesh;
    CubeMesh* mesh2;
    
    for (size_t i = 0; i < Mesh::meshCount; i++)
    {
        if ((i + 1) >= Mesh::meshCount)
        {
            break;
        }

        //Current Mesh
        mesh = (CubeMesh*)Mesh::meshes[i];
        if (!mesh)
        {
            continue;
        }
        for (size_t j = i + 1; j < Mesh::meshCount; j++)
        {
            // Other Mesh
            mesh2 = (CubeMesh*)Mesh::meshes[j];
            if (!mesh2) {
                continue;
            }
            if (Collision(*mesh, *mesh2))
            {
                mesh->color = RGB::pink;
                mesh2->color = RGB::pink;
            }
        }
    }
}

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

    if (planet) {
        float planetRotationSpeed = 1.5 * PI / 180 * deltaTime;
        planet->rotation = Matrix3x3::RotX(-planetRotationSpeed) * Matrix3x3::RotY(planetRotationSpeed + 0.000001) * planet->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
    }
    
    for (size_t i = 0; i < Mesh::meshes.size(); i++)
    {
        if (Mesh::meshes[i])
        {
            Mesh::meshes[i]->position += Mesh::meshes[i]->position  * (-deltaTime / 10.0);
        }
    }

    DetectCollisions();
}
#endif