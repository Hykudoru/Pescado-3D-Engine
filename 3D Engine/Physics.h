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
    static bool raycasting;
    static bool gravity;
};
bool Physics::collisionDetection = true;
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
/*
class Component
{

};

class GameObject
{
public:
    static List<GameObject*> gameObjects;
    static int gameObjectCount;
    List<Component> components;
    Transform* transform;
    Mesh* mesh;
    RigidBody* rigidBody;

    GameObject(Mesh& mesh, RigidBody& rigidBody)
    {
        this->mesh = &mesh;
        this->rigidBody = &rigidBody;
        GameObject::gameObjects.emplace(gameObjects.begin() + gameObjectCount++, this);
    }

    ~GameObject()
    {
        if (transform)
        {
            delete transform;
        }
        if (mesh)
        {
            delete mesh;
        }
        if (rigidBody)
        {
            delete rigidBody;
        }
    }

    static void Update()
    {
        for (size_t i = 0; i < gameObjectCount; i++)
        {
            if (gameObjects[i]->rigidBody)
            {
                gameObjects[i]->Update();
            }
        }
    }
};*/

class RigidBody
{
public:
    float mass = 1;
    bool isKinematic = false;
    Vec3 velocity = Vec3::zero;
    Vec3 angularVelocity = Vec3::zero;
    Vec3 position = Vec3::zero;
    Matrix3x3 rotation = Identity3x3;

    //Collider collider;
};


class Collider
{
public:
    bool isStatic = false;
};

class PhysicsObject: public CubeMesh, public RigidBody, public Collider, public ManagedObjectPool<PhysicsObject>
{
public:
    PhysicsObject():ManagedObjectPool<PhysicsObject>(this)
    {

    }
};

// Oriented Bounding Box (OBB) with Separating Axis Theorem (SAT) algorithm
bool Collision(PhysicsObject& physObj1, PhysicsObject& physObj2, bool resolve = true)
{
    CubeMesh& mesh1 = (CubeMesh&)(physObj1);
    CubeMesh& mesh2 = (CubeMesh&)(physObj2);
    bool gap = false;
    float minOverlap = 0;
    Vec3 minOverlapAxis;

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
    for (size_t i = 0; i < 2; i++)
    {
        for (size_t j = 0; j < physObj1Normals.size(); j++)
        {
            Vec3 n1 = physObj1Normals[j];
            Range rangeA = ProjectVertsOntoAxis(physObj1Verts.data(), physObj1Verts.size(), n1);
            Range rangeB = ProjectVertsOntoAxis(physObj2Verts.data(), physObj2Verts.size(), n1);
            gap = !((rangeA.max >= rangeB.min && rangeB.max >= rangeA.min));// || (mesh1Range.max < mesh2Range.min && mesh2Range.max < mesh1Range.min));
            if (gap) {
                break;
            }
            
            //Compare and cache minimum projection distance and axis for later use if needed for collision resolution.
            float potentialMinOverlap = 0;
            Vec3 dir = n1;
            if (rangeA.max > rangeB.max) {
                potentialMinOverlap = rangeB.max - rangeA.min;
                dir *= -1.0;// Reverse push direction since object B is in front of object A when comparing to object A's normal
            }
            else {
                potentialMinOverlap = rangeA.max - rangeB.min;
            }

            if (j == 0) 
            {   //Initialize minimum projection distance and axis
                minOverlap = potentialMinOverlap;
                minOverlapAxis = dir;
            }
            else if (potentialMinOverlap < minOverlap)
            {
                minOverlap = potentialMinOverlap;
                minOverlapAxis = dir;
            }
        }

        if (gap) {
            break;
        }
        //Swap and repeat once more for the other mesh
        physObj1Verts.swap(physObj2Verts);
        physObj1Normals.swap(physObj2Normals);
    }

    // Step 3: Must continue searching for possible 3D Edge-Edge collision
    if (!gap) 
    {
        for (size_t i = 0; i < physObj1Normals.size(); i++)
        {
            Vec3 nA = physObj1Normals[i];
            for (size_t j = 0; j < physObj2Normals.size(); j++)
            {
                Vec3 axis;
                
                Vec3 nB = physObj2Normals[j];
                //Make sure normals are not the same before using them to calculate the cross product (otherwise the axis would be <0, 0, 0>).
                if (nA.x == nB.x && nA.y == nB.y && nA.z == nB.z)
                {
                    if ((j + 1) >= physObj2Normals.size()) {
                        nB = physObj2Normals[j - 1];
                    }
                    else {
                        nB = physObj2Normals[j + 1];
                    }
                }
                // Search for possible 3D Edge-Edge collision
                axis = CrossProduct(nA, nB);
                Range rangeA = ProjectVertsOntoAxis(physObj1Verts.data(), physObj1Verts.size(), axis);
                Range rangeB = ProjectVertsOntoAxis(physObj2Verts.data(), physObj2Verts.size(), axis);
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

    if (!gap && resolve)
    {
        Vec3 offset = minOverlapAxis * minOverlap;

        bool neitherStatic = !physObj1.isStatic && !physObj2.isStatic;
        if (neitherStatic)
        {
            offset /= 2.0;
            physObj1.root->position += offset;
            physObj2.root->position -= offset;

            return !gap;
        }

        //Only one is movable at this stage

        if (physObj1.isStatic) {
            physObj2.root->position -= offset;
        }
        else
        {
            physObj1.root->position += offset;
        }
    }

    if (gap)
    {
        return false;
    }

    return !gap;//No gap = collision
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

            if (Collision(*physObj1, *physObj2))
            {   
                //mesh1->color = RGB::pink;
                //mesh2->color = RGB::pink;
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

    //for (size_t i = 0; i < ManagedObjectPool<PhysicsObject>::count; i++)
    //{
      //  PhysicsObject* obj = ManagedObjectPool<PhysicsObject>::objects[i];
        //if (!obj->isStatic)
        //{
          //  Transform* t = ((Transform*)obj);
           //t->position += gravity * deltaTime;
           //t->rotation = Identity3x3;
        //}
    //}

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