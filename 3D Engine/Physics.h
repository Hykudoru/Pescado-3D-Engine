#pragma once
#include <GLFW/glfw3.h>
#include <math.h>
#include <Matrix.h>
#include <Graphics.h>
using namespace std;

#ifndef PHYSICS_H
#define PHYSICS_H

class Physics
{
public:
    static bool collisionDetection;
};
bool Physics::collisionDetection = true;

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
//PhysicsObject physicsCube1 = PhysicsObject(new CubeMesh());// PhysicsObject();
//PhysicsObject physicsCube2 = PhysicsObject(new CubeMesh());// PhysicsObject();

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
        if (k == 0)
        {
            min = dist;
            max = dist;
        }
        else if (dist < min) {
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
    if (!mesh1.vertices || !mesh2.vertices) {
        return false;
    }
    List<Vec3> mesh1Verts = *mesh1.vertices;
    List<Vec3> mesh2Verts = *mesh2.vertices;
    List<Vec3> mesh1Normals = mesh1.WorldXYZNormals();
    List<Vec3> mesh2Normals = mesh2.WorldXYZNormals();
    for (size_t i = 0; i < mesh1Verts.size(); i++){ mesh1Verts[i] = (Vec3)(mesh1.TRS() * ((Vec4)mesh1Verts[i])); }//Convert to world coordinates
    for (size_t i = 0; i < mesh2Verts.size(); i++){ mesh2Verts[i] = (Vec3)(mesh2.TRS() * ((Vec4)mesh2Verts[i])); }//Convert to world coordinates

    // Collision detection stops if at any time a gap is found.
    // Step 1: Project both meshes onto Mesh A's normal axes.
    // Step 2: Project both meshes onto Mesh B's normal axes.
    for (size_t i = 0; i < 2; i++)
    {
        for (size_t j = 0; j < mesh1Normals.size(); j++)
        {
            Vec3 n1 = mesh1Normals[j];
            Range rangeA = ProjectVertsOntoAxis(mesh1Verts, n1);
            Range rangeB = ProjectVertsOntoAxis(mesh2Verts, n1);
            gap = !((rangeA.max >= rangeB.min && rangeB.max >= rangeA.min));// || (mesh1Range.max < mesh2Range.min && mesh2Range.max < mesh1Range.min));
            if (gap) {
                break;
            }
        }
        if (gap) {
            break;
        }
        //Swap and repeat once more for the other mesh
        mesh1Verts.swap(mesh2Verts);
        mesh1Normals.swap(mesh2Normals);
    }

    // Step 3: Must continue searching for possible 3D Edge-Edge collision
    if (!gap) 
    {
        for (size_t i = 0; i < mesh1Normals.size(); i++)
        {
            for (size_t j = 0; j < mesh2Normals.size(); j++)
            {
                Vec3 axis;
                Vec3 nA = mesh1Normals[i];
                Vec3 nB = mesh2Normals[j];
                //Make sure normals are not the same before using them to calculate the cross product (otherwise the axis would be <0, 0, 0>).
                if (nA.x == nB.x && nA.y == nB.y && nA.z == nB.z)
                {
                    if ((j + 1) >= mesh2Normals.size()) {
                        nB = mesh2Normals[j - 1];
                    }
                    else {
                        nB = mesh2Normals[j + 1];
                    }
                }

                axis = CrossProduct(nA, nB);
                Range rangeA = ProjectVertsOntoAxis(mesh1Verts, axis);
                Range rangeB = ProjectVertsOntoAxis(mesh2Verts, axis);
                gap = !((rangeA.max >= rangeB.min && rangeB.max >= rangeA.min));// || (mesh1Range.max < mesh2Range.min && mesh2Range.max < mesh1Range.min));
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
    // How it works: Meshes A, B, C, D, E...
    // Compare A:B, A:C, A:D, A:E
    // Compare B:C, B:D, B:E
    // Compare C:D, C:E
    // Compare D:E
    for (size_t i = 0; i < Mesh::meshCount; i++)
    {
        // exit if this is the last mesh
        if ((i + 1) >= Mesh::meshCount) {
            break;
        }

        // Current Mesh
        CubeMesh* mesh = (CubeMesh*)Mesh::meshes[i];
        if (!mesh)
        {
            continue;
        }
        for (size_t j = i + 1; j < Mesh::meshCount; j++)
        {
            // Next Mesh
            CubeMesh* mesh2 = (CubeMesh*)Mesh::meshes[j];
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
           //Mesh::meshes[i]->position += Mesh::meshes[i]->position  * (-deltaTime / 10.0);
        }
    }

    if (Physics::collisionDetection)
    {
        DetectCollisions();
    }
}
#endif