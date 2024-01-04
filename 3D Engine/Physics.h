#pragma once
#include <GLFW/glfw3.h>
#include <math.h>
#include <Matrix.h>
#include <Graphics.h>
#include <Utility.h>
using namespace std;

#ifndef PHYSICS_H
#define PHYSICS_H
extern bool DEBUGGING;
extern GLFWwindow* window;
Vec3 gravity = Vec3(0, -9.81, 0);
const float defaultAcceleration = 50;
float accel = defaultAcceleration;
float decel = -5;
float rotateSpeed = PI / 2;
bool isKinematic = false;
bool dampenersActive = true;
Vec3 moveDir = Vec3(0, 0, 0);
Vec3 velocity = Vec3(0, 0, 0);

extern Mesh* planet;
extern Mesh* spaceShip;
extern Mesh* spaceShip2;
extern Mesh* spaceShip3;

class Physics
{
public:
    static bool collisionDetection;
    static bool dynamics;
    static bool raycasting;
    static bool gravity;
};
bool Physics::collisionDetection = true;
bool Physics::dynamics = true;
bool Physics::raycasting = false;
bool Physics::gravity = false;

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

class RigidBody
{
public:
    float mass = 1;
    bool isKinematic = false;
    Vec3 velocity = Vec3::zero;
    Vec3 angularVelocity = Vec3::zero;
};


class BoxCollider: public CubeMesh
{
public:
    bool isStatic = false;
};

class PhysicsObject: public RigidBody, public BoxCollider, public ManagedObjectPool<PhysicsObject>
{
public:
    PhysicsObject():ManagedObjectPool<PhysicsObject>(this) {}
};

struct CollisionInfo
{
    bool gap = false;
    float minOverlap = 0;
    Vec3 minOverlapAxis = Vec3::zero;
};
// Oriented Bounding Box (OBB) with Separating Axis Theorem (SAT) algorithm
bool OBBSATCollision(PhysicsObject& physObj1, PhysicsObject& physObj2, CollisionInfo& collisionInfo, bool resolve = true)
{
    collisionInfo = CollisionInfo();

    if (!physObj1.vertices || !physObj2.vertices) {
        return false;
    }
    if (physObj1.isStatic && physObj2.isStatic)
    {
        return false;
    }
    List<Vec3> physObj1Verts = physObj1.WorldVertices();
    List<Vec3> physObj2Verts = physObj2.WorldVertices();
    List<Vec3> physObj1Normals = List<Vec3>{ physObj1.Right(), physObj1.Up(), physObj1.Forward() };// mesh1.WorldXYZNormals();
    List<Vec3> physObj2Normals = List<Vec3>{ physObj2.Right(), physObj2.Up(), physObj2.Forward() }; //mesh2.WorldXYZNormals();

    // Note: Collision detection stops if at any time a gap is found.
    // Note: Cache the minimum distance projection and axis for later use to resolve the collision if needed.
    // Step 1: Project both meshes onto Mesh A's normal axes.
    // Step 2: Project both meshes onto Mesh B's normal axes.
    //Initialize minimum projection distance and axis
    for (size_t i = 0; i < 2; i++)
    {
        auto normals = i == 0 ? &physObj1Normals : &physObj2Normals;
        for (size_t ii = 0; ii < normals->size(); ii++)
        {
            Vec3 axis = (*normals)[ii];
            Range rangeA = ProjectVertsOntoAxis(physObj1Verts.data(), physObj1Verts.size(), axis);
            Range rangeB = ProjectVertsOntoAxis(physObj2Verts.data(), physObj2Verts.size(), axis);
            collisionInfo.gap = !((rangeA.max >= rangeB.min && rangeB.max >= rangeA.min));// || (mesh1Range.max < mesh2Range.min && mesh2Range.max < mesh1Range.min));
            if (collisionInfo.gap) {
                return false;
            }

            //Compare and cache minimum projection distance and axis for later use if needed for collision resolution.
            float potentialMinOverlap = 0;
            if (rangeA.max > rangeB.max) {
                potentialMinOverlap = rangeB.max - rangeA.min;
                axis *= -1.0;// Reverse push direction since object B is behind object A and we will always push A backwards and B forwards.
            } else {
                potentialMinOverlap = rangeA.max - rangeB.min;
            }

            if (i == 0)
            {
                collisionInfo.minOverlap = potentialMinOverlap;
                collisionInfo.minOverlapAxis = axis;
            }
            else if (potentialMinOverlap < collisionInfo.minOverlap)
            {
                collisionInfo.minOverlap = potentialMinOverlap;
                collisionInfo.minOverlapAxis = axis;
            }
        }
    }

    // Step 3: Must continue searching for possible 3D Edge-Edge collision
    if (!collisionInfo.gap)
    {
        for (size_t i = 0; i < physObj1Normals.size(); i++)
        {
            Vec3 nA = physObj1Normals[i];
            for (size_t j = 0; j < physObj2Normals.size(); j++)
            {
                Vec3 nB = physObj2Normals[j];
                
                //Make sure normals are not the same before using them to calculate the cross product (otherwise the axis would be <0, 0, 0>).
                float dot = DotProduct(nA, nB);
                bool sameAxis = dot >= 1.0 || dot <= -1.0;
                if (sameAxis)
                {
                    if ((j + 1) >= physObj2Normals.size()) {
                        nB = physObj2Normals[j - 1];
                    }
                    else {
                        nB = physObj2Normals[j + 1];
                    }
                }
                
                // Search for possible 3D Edge-Edge collision
                Vec3 axis = CrossProduct(nA, nB);
                Range rangeA = ProjectVertsOntoAxis(physObj1Verts.data(), physObj1Verts.size(), axis);
                Range rangeB = ProjectVertsOntoAxis(physObj2Verts.data(), physObj2Verts.size(), axis);
                collisionInfo.gap = !((rangeA.max >= rangeB.min && rangeB.max >= rangeA.min));// || (mesh1Range.max < mesh2Range.min && mesh2Range.max < mesh1Range.min));
                if (collisionInfo.gap) {
                    return false;
                }
               /* To-Do...
                //Compare and cache minimum projection distance and axis for later use if needed for collision resolution.
                float potentialMinOverlap = 0;
                if (rangeA.max > rangeB.max) {
                    potentialMinOverlap = rangeB.max - rangeA.min;
                    axis *= -1.0;// Reverse push direction since object B is behind object A and we will always push A backwards and B forwards.
                } 
                else {
                    potentialMinOverlap = rangeA.max - rangeB.min;
                }

                if (potentialMinOverlap < collisionInfo.minOverlap)
                {
                    collisionInfo.minOverlap = potentialMinOverlap;
                    collisionInfo.minOverlapAxis = axis;
                }*/
            }
        }
    }

    if (!collisionInfo.gap && resolve)
    {
        Vec3 offset = collisionInfo.minOverlapAxis * collisionInfo.minOverlap;

        bool neitherStatic = !physObj1.isStatic && !physObj2.isStatic;
        if (neitherStatic)
        {
            offset *= 0.5;
            physObj1.root->position -= (offset*1.01);
            physObj2.root->position += (offset*1.01);

            return !collisionInfo.gap;
        }

        //Only one is movable at this stage

        if (physObj1.isStatic) {
            physObj2.root->position += offset;
        }
        else
        {
            physObj1.root->position -= offset;
        }
        
    }

    if (collisionInfo.gap)
    {
        return false;
    }

    return !collisionInfo.gap;//No gap = collision
}

void DetectCollisions()
{
    // How nested loop algorithm works: 
    // Gets meshes A, B, C, D, E...
    // Compare A:B, A:C, A:D, A:E
    // Compare B:C, B:D, B:E
    // Compare C:D, C:E
    // Compare D:E
    for (size_t i = 0; i < ManagedObjectPool<PhysicsObject>::count; i++)
    {
        // exit if this is the last Collider
        if ((i + 1) >= ManagedObjectPool<PhysicsObject>::count) {
            break;
        }
        
        // Current Collider
        PhysicsObject* physObj1 = ManagedObjectPool<PhysicsObject>::objects[i];

        for (size_t j = i + 1; j < ManagedObjectPool<PhysicsObject>::count; j++)
        { 
            // Next Collider
            PhysicsObject* physObj2 = ManagedObjectPool<PhysicsObject>::objects[j];
            
            if (physObj1->isStatic && physObj2->isStatic) {
                continue;
            }

            CollisionInfo collisionInfo;
            if (OBBSATCollision(*physObj1, *physObj2, collisionInfo))
            {   
                if (Physics::dynamics)
                {
                    
                    if (physObj1->isKinematic || physObj2->isKinematic) {
                        continue;
                    }
                    /*
                    Although static objects themselves are not effected by momentum
                    transfers, their velocity variable may still be updating from new collisions.
                    Consequently, objects touching a static collider would be effected, so the
                    velocity is zeroed out to prevent this.
                    */
                    if (physObj1->isStatic) 
                    {
                        physObj1->velocity = Vec3::zero;
                    }
                    else if (physObj2->isStatic) 
                    {
                        physObj2->velocity = Vec3::zero;
                    }

                    float m1 = physObj1->mass;
                    float m2 = physObj2->mass;
                    Vec3 v1 = physObj1->velocity;
                    Vec3 v2 = physObj2->velocity;

                    /* Elastic collision (conserves both momentum and kinetic energy)
                    Conservation Momentum: m1*v1 + m2*v2 = m1*v1' + m2*v2'
                    Conservation Kinetic Energy: v1 + v1' = v2 + v2'
                    The eq below were solved from the system of eq above.*/
                    Vec3 v1Final = (v1 * m1 + v2 * m2 * 2.0 - v1 * m2) * (1.0 / (m1 + m2));
                    Vec3 v2Final = v1 + v1Final - v2;

                    physObj1->velocity = v1Final;
                    physObj2->velocity = v2Final;
                }
            }
        }
    }
}


static void Physics()
{
    Camera* cam;
    if (CameraSettings::outsiderViewPerspective)
    {
        cam = Camera::projector;
    }
    else {
        cam = Camera::main;
    }
    //---------- Physics Update ------------
    if (isKinematic) 
    {
        cam->position += moveDir * accel * deltaTime;
    }
    else 
    {
        velocity += moveDir * accel * deltaTime;
        if (Physics::gravity) {
            velocity += gravity * deltaTime;
        }
        else if (dampenersActive) {
            velocity += velocity * decel * deltaTime;
        }
        if (velocity.SqrMagnitude() < 0.0001) {
            velocity = Vec3::zero;
        }
        cam->position += velocity * deltaTime;
    }

    if (planet) {
        float planetRotationSpeed = (1.5 * PI / 180) * deltaTime;
        planet->rotation = Matrix3x3::RotX(-planetRotationSpeed) * Matrix3x3::RotY(planetRotationSpeed + 0.000001) * planet->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
    }

    if (spaceShip)
    {
        float shipRotationSpeed = (10 * PI / 180) * deltaTime;
        spaceShip->rotation = Matrix3x3::RotY(-shipRotationSpeed) * spaceShip->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
        spaceShip->position += spaceShip->Forward() * 20 * deltaTime;
    }
    if (spaceShip2)
    {
        float shipRotationSpeed = (5 * PI / 180) * deltaTime;
        spaceShip2->rotation = Matrix3x3::RotZ(shipRotationSpeed) * Matrix3x3::RotY(-shipRotationSpeed) * spaceShip2->rotation;// *spaceShip2->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
        spaceShip2->position += spaceShip2->Forward() * 10 * deltaTime;
    }
    if (spaceShip3)
    {
        float shipRotationSpeed = (20*PI / 180) * deltaTime;
        spaceShip3->rotation = Matrix3x3::RotY(shipRotationSpeed) * Matrix3x3::RotZ(shipRotationSpeed) * spaceShip3->rotation;// *spaceShip2->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
        spaceShip3->position += spaceShip3->Forward() * 15 * deltaTime;

    }

    if (Physics::dynamics)
    {
        for (size_t i = 0; i < ManagedObjectPool<PhysicsObject>::count; i++)
        {
            PhysicsObject* obj = ManagedObjectPool<PhysicsObject>::objects[i];
            if (!obj->isStatic)
            {
                if (Physics::gravity) {
                    obj->velocity += gravity * deltaTime;
                }
                obj->position += obj->velocity * deltaTime;
            }
        }
    }

    if (Physics::collisionDetection)
    {
        DetectCollisions();
    }
    
    if (Physics::raycasting)
    {
        for (size_t i = 0; i < Mesh::objects.size(); i++)
        {
            auto triangles = Mesh::objects[i]->MapVertsToTriangles();
            for (size_t j = 0; j < triangles->size(); j++)
            {
                Triangle worldSpaceTri = (*triangles)[j];
                Triangle projectedTri = (*triangles)[j];
                for (size_t k = 0; k < 3; k++)
                {
                    worldSpaceTri.verts[k] = Mesh::objects[i]->TRS() * worldSpaceTri.verts[k];
                    projectedTri.verts[k] = ProjectionMatrix() * Camera::main->TRInverse() * worldSpaceTri.verts[k];
                }
                //------------------Ray casting (world & view space)--------------------------
                Vec3 lineStart = Camera::cameras[2]->position;
                Vec3 lineEnd = lineStart + Camera::cameras[2]->Forward();// *abs(farClippingPlane);
                Vec3 pointOfIntersection;
                if (LinePlaneIntersecting(lineStart, lineEnd, worldSpaceTri, &pointOfIntersection))
                {
                    Matrix4x4 matrix = ProjectionMatrix() * Camera::main->TRInverse();

                    Vec4 pointOfIntersection_p = matrix * pointOfIntersection;
                    if (PointInsideTriangle(pointOfIntersection_p, projectedTri.verts))
                    {
                        // ---------- Debugging -----------
                        //Vec4 pointOfIntersectionProj = projectionMatrix * worldToViewMatrix * pointOfIntersection;
                        Vec4 from_p = matrix * lineStart;
                        Vec4 to_p = matrix * (pointOfIntersection);// +lineEnd);
                        lineBuffer->emplace_back(Line(from_p, pointOfIntersection_p));
                        pointBuffer->emplace_back(Point(pointOfIntersection_p, RGB::gray, 5));

                        // Reflect
                        Vec3 n = worldSpaceTri.Normal();
                        Vec3 v = (pointOfIntersection - lineStart);
                        Vec3 reflection = Reflect(v, n);
                        lineBuffer->emplace_back(Line(pointOfIntersection_p, (Vec3)(matrix * (pointOfIntersection + reflection)), RGB::red));

                        // Project
                        Vec3 vecPlane = ProjectOnPlane(v, n);
                        lineBuffer->emplace_back(Line(pointOfIntersection_p, (Vec3)(matrix * (pointOfIntersection + vecPlane)), RGB::black));

                        projectedTri.color = RGB::white;
                        if (DEBUGGING) { std::cout << (projectedTri.mesh) << endl; }// delete projectedTri.mesh; }
                    }
                }
            }
        }
    }

    if (DEBUGGING) 
    { 
        std::cout << "--------PHYSICS-------" << endl;

        string onoff = Physics::collisionDetection ? "On" : "Off";
        std::cout << "Collisions: " << onoff << " (press P)" << endl;

        onoff = Physics::gravity ? "On" : "Off";
        std::cout << "Gravity: " << onoff << " (press G)" << endl;

        std::cout << "--------PLAYER-------" << endl;

        onoff = isKinematic ? "On" : "Off";
        std::cout << "Kinematic: " << onoff << " (press X)" << endl;

        onoff = dampenersActive ? "On" : "Off";
        std::cout << "Inertial Dampeners: " << onoff << " (press Z)" << endl;

        std::cout << "Position: (" << Camera::main->position.x << ", " << Camera::main->position.y << ", " << Camera::main->position.z << ")" << endl;
        std::cout << "Velocity: <" << velocity.x << ", " << velocity.y << ", " << velocity.z << ">" << endl;
    }
}
#endif