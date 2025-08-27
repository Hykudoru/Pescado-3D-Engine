#pragma once
#include <GLFW/glfw3.h>
#include <math.h>
#include <Functional>
#include <Matrix.h>
#include <Graphics.h>
#include <Utility.h>
#include <OctTree.h>
using namespace std;
/*TO-DO
*
* // OPTIMIZATIONS
* - Revisit sphere-plane collisions (possible performance optimization)
*
* // ISSUES
* - Fix Physics Object collider parenting issue
*/

class PhysicsObject;

#ifndef PHYSICS_H
#define PHYSICS_H
extern bool DEBUGGING;
Vec3 gravity = Vec3(0, -9.81, 0);
const float defaultAcceleration = 50;
float accel = defaultAcceleration;
float decel = -5;
float rotateSpeed = PI / 2;
bool isKinematic = false;
bool dampenersActive = true;
Vec3 moveDir = Vec3(0, 0, 0);
Vec3 velocity = Vec3(0, 0, 0);

class Physics
{
public:
    static bool time;
    static bool collisionDetection;
    static bool dynamics;
    static bool raycasting;
    static bool raycastDebugging;
    static bool gravity;
    static bool octTree;
};
bool Physics::time = true;
bool Physics::collisionDetection = true;
bool Physics::dynamics = true;
bool Physics::raycasting = false;
bool Physics::raycastDebugging = false;
bool Physics::gravity = false;
bool Physics::octTree = true;

double _engineDeltaTime = 0;
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

    _engineDeltaTime = deltaTime;
    if (!Physics::time) {

        deltaTime = 0;
    }
}

class RigidBody
{
public:
    Vec3 centerOfMass = Vec3::zero;
    float mass = 1;
    bool isKinematic = false;
    Vec3 velocity = Vec3::zero;
    Vec3 angVelAxis = Vec3::zero;
    float angVelSpeed = 0;

    Vec3 AngularVelocity()
    {
        return angVelAxis * angVelSpeed;
    }
};

class Collider : public Component, public Transform, public  ManagedObjectPool<Collider>
{
public:
    Mesh* mesh = nullptr;
    bool isStatic = false;
    bool isTrigger = false;
    float coefficientRestitution = 0.5;
    bool flagged = false;
    std::function<void(Collider*)> OnCollision = [](Collider* collider) {};

    Collider(bool isStatic = false) : ManagedObjectPool<Collider>(this)
    {
        this->isStatic = isStatic;
    }

    virtual ~Collider()
    {
        delete mesh;
    }

    List<Triangle>* MapVertsToTriangles()
    {
        return mesh->MapVertsToTriangles();
    }

    List<Vec3> WorldVertices()
    {
        return mesh->WorldVertices();
    }

    List<Vec3> Vertices()
    {
        return mesh->vertices;
    }
};

class BoxCollider : public Collider, public ManagedObjectPool<BoxCollider>
{
public:
    BoxCollider(bool isStatic = false) : Collider(isStatic), ManagedObjectPool<BoxCollider>(this)
    {
        mesh = new CubeMesh();
        mesh->SetColor(Color::red);
        mesh->SetParent(this);
        mesh->SetVisibility(false);
    }
};

class SphereCollider : public Collider, public ManagedObjectPool<SphereCollider>
{
    float radius = 1;
public:

    SphereCollider(bool isStatic = false) : Collider(isStatic), ManagedObjectPool<SphereCollider>(this)
    {
        mesh = LoadMeshFromOBJFile("Sphere.obj");
        mesh->SetColor(Color::red);
        mesh->SetParent(this);
        mesh->SetVisibility(false);
    }

    float Radius()
    {
        return Scale().x;
    }
};

class PlaneCollider : public Collider, public ManagedObjectPool<PlaneCollider>
{
public:
    Vec3 normal;
    PlaneCollider(Vec3 normal, bool isStatic = false) : Collider(isStatic), ManagedObjectPool<PlaneCollider>(this)
    {
        mesh = new PlaneMesh();
        this->normal = normal;
        mesh->SetColor(Color::red);
        mesh->SetParent(this);
        mesh->SetVisibility(false);
    }
};

class PhysicsObject : public Transform, public RigidBody, public ManagedObjectPool<PhysicsObject>
{
public:
    Collider* collider;
    Mesh* mesh;

    PhysicsObject(Mesh* mesh, BoxCollider* collider) : ManagedObjectPool<PhysicsObject>(this)
    {
        this->localPosition = mesh->localPosition;
        this->localRotation = mesh->localRotation;
        this->localScale = mesh->localScale;
        mesh->localPosition = Vec3::zero;
        mesh->localRotation = Matrix3x3::identity;
        mesh->localScale = Vec3::one;
        SetCollider(collider);
        SetMesh(mesh);

        delete collider->mesh;
        collider->mesh = mesh;
        collider->mesh->bounds->color = Color::red;
    }

    PhysicsObject(Mesh* mesh, SphereCollider* collider) : ManagedObjectPool<PhysicsObject>(this)
    {
        this->localPosition = mesh->localPosition;
        this->localRotation = mesh->localRotation;
        this->localScale = mesh->localScale;
        mesh->localPosition = Vec3::zero;
        mesh->localRotation = Matrix3x3::identity;
        mesh->localScale = Vec3::one;
        SetCollider(collider);
        SetMesh(mesh);
        collider->mesh->bounds->color = Color::red;
        mesh->bounds->color = Color::blue;
    }

    PhysicsObject(float scale, Vec3 position, Matrix3x3 rotation, Mesh* mesh, BoxCollider* collider) : ManagedObjectPool<PhysicsObject>(this)
    {
        this->localPosition = position;
        this->localRotation = rotation;
        this->localScale = Vec3(scale, scale, scale);
        mesh->localPosition = Vec3::zero;
        mesh->localRotation = Matrix3x3::identity;
        mesh->localScale = Vec3::one;
        SetCollider(collider);
        SetMesh(mesh);
        delete collider->mesh;
        collider->mesh = mesh;
        collider->mesh->bounds->color = Color::red;
    }

    PhysicsObject(float scale, Vec3 position, Matrix3x3 rotation, Mesh* mesh, SphereCollider* collider) : ManagedObjectPool<PhysicsObject>(this)
    {
        this->localPosition = position;
        this->localRotation = rotation;
        this->localScale = Vec3(scale, scale, scale);
        mesh->localPosition = Vec3::zero;
        mesh->localRotation = Matrix3x3::identity;
        mesh->localScale = Vec3::one;
        SetCollider(collider);
        SetMesh(mesh);
        collider->mesh->bounds->color = Color::red;
        mesh->bounds->color = Color::blue;
    }

    virtual ~PhysicsObject()
    {
        delete mesh;
        collider->mesh = nullptr;
        delete collider;
    }

    void SetCollider(Collider* collider)
    {
        if (collider)
        {
            //delete this->collider;
            this->collider = collider;
            this->collider->object = this;
            this->collider->SetParent(this, false);
        }
    }

    void SetMesh(Mesh* mesh)
    {
        if (mesh)
        {
            //delete this->mesh;
            this->mesh = mesh;
            this->mesh->object = this;
            this->mesh->SetParent(collider, false);
        }
    }

    void IsTrigger(bool condition)
    {
        collider->isTrigger = condition;
        if (condition)
        {
            isKinematic = true;
        }
    }

    bool IsTrigger()
    {
        return collider->isTrigger;
    }
};

struct CollisionInfo
{
    bool colliding = false;
    Vec3 lineOfImpact = Vec3::zero;
    Vec3 pointOfContact = Vec3::zero;
};

struct BoxCollisionInfo : public CollisionInfo
{
    float minOverlap = 0;
    Vec3 minOverlapAxis = Vec3::zero;
};

void ResolveCollision(Collider& colliderA, Collider& colliderB, Vec3& penDepth);

void CalculateCollision(Vec3& lineOfImpact, float& m1, float& m2, Vec3& v1, Vec3& v2, float e = 1.0)
{
    /* Elastic collision (conserves both momentum and kinetic energy)
    Conservation Momentum: m1*v1 + m2*v2 = m1*v1' + m2*v2'
    Conservation Kinetic Energy: v1 + v1' = v2 + v2'
    Coefficient of Restitution: e = v2'-v1' / v1-v2
        e = 1 Perfectly elastic
        0 < e < 1 inelastic
        e = 0 Perfectly inelastic
    */
    lineOfImpact = lineOfImpact.Normalized();
    Vec3 v1LineOfImpact = lineOfImpact * DotProduct(v1, lineOfImpact);
    Vec3 v2LineOfImpact = lineOfImpact * DotProduct(v2, lineOfImpact);
    Vec3 _ = (m1 * v1LineOfImpact) + (m2 * v2LineOfImpact);
    Vec3 v1LineOfImpactFinal = (_ - (m2 * e * (v1LineOfImpact - v2LineOfImpact))) * (1.0 / (m1 + m2));
    Vec3 v2LineOfImpactFinal = (_ - (m1 * v1LineOfImpactFinal))* (1.0 / m2);
    Vec3 v1PerpendicularFinal = (v1 - v1LineOfImpact);//Perpendicular Velocity is the same before and after impact
    Vec3 v2PerpendicularFinal = (v2 - v2LineOfImpact);//Perpendicular Velocity is the same before and after impact
    Vec3 v1Final = v1LineOfImpactFinal + v1PerpendicularFinal;
    Vec3 v2Final = v2LineOfImpactFinal + v2PerpendicularFinal;

    v1 = v1Final;
    v2 = v2Final;
}

void CalculateStaticCollision(Vec3& lineOfImpact, Vec3& v1, float e = 1.0)
{
    lineOfImpact = lineOfImpact.Normalized();
    Vec3 v1LineOfImpact = lineOfImpact * DotProduct(v1, lineOfImpact);
    Vec3 v1PerpendicularFinal = (v1 - v1LineOfImpact);//Perpendicular Velocity is the same before and after impact
    v1 = (-e * v1LineOfImpact) + v1PerpendicularFinal;
}

bool SpherePlaneColliding(SphereCollider& sphere, PlaneCollider& plane, CollisionInfo& collisionInfo, bool resolve = true)
{
    float radius = sphere.Radius();
    Vec3 sphereCenter = sphere.Position();
    Vec3 v = sphereCenter - plane.Position();
    Vec3 normal = plane.normal;
    Vec3 vPerp = normal * (DotProduct(v, normal));//ProjectOnPlane(v, plane.plane.normal);
    Vec3 closestPointOnPlane = sphereCenter - vPerp;

    if ((closestPointOnPlane - sphereCenter).SqrMagnitude() < radius * radius)
    {
        collisionInfo.colliding = true;
        Vec3 pointOnSphere = ClosestPointOnSphere(sphereCenter, radius, closestPointOnPlane);
        collisionInfo.pointOfContact = pointOnSphere;
        collisionInfo.lineOfImpact = normal * -1.0;
        if (resolve)
        {
            Vec3 offset = pointOnSphere - closestPointOnPlane;//overlapping
            sphere.Root().localPosition -= offset;
        }
    }

    if (Graphics::debugPlaneCollisions)
    {
        Vec3 vProj = ProjectOnPlane(v, normal);
        Line::AddWorldLine(Line(plane.Position(), plane.Position() + normal, Color::gray));
        Line::AddWorldLine(Line(sphereCenter, closestPointOnPlane, Color::red));
        Point::AddWorldPoint(Point(sphereCenter, Color::gray, 10));
        Point::AddWorldPoint(Point(closestPointOnPlane, Color::red, 10));
    }

    return collisionInfo.colliding;
}

bool SpheresColliding(SphereCollider& sphere1, SphereCollider& sphere2, CollisionInfo& collisionInfo, bool resolve = true)
{
    float radius1 = sphere1.Radius();
    float radius2 = sphere2.Radius();
    Vec3 sphere1Pos = sphere1.Position();
    Vec3 sphere2Pos = sphere2.Position();
    float sumRadii = (radius1 + radius2);
    collisionInfo.colliding = (sphere2Pos - sphere1Pos).SqrMagnitude() < (sumRadii * sumRadii);
    if (collisionInfo.colliding)
    {
        Vec3 pointOnSphere1 = ClosestPointOnSphere(sphere1Pos, radius1, sphere2Pos);
        Vec3 pointOnSphere2 = ClosestPointOnSphere(sphere2Pos, radius2, sphere1Pos);
        Vec3 offset = (pointOnSphere1 - pointOnSphere2);
        collisionInfo.lineOfImpact = offset;
        collisionInfo.pointOfContact = (pointOnSphere1 + pointOnSphere2) * 0.5;
        if (resolve)
        {
            ResolveCollision(sphere1, sphere2, offset);
        }
    }

    if (Graphics::debugSphereCollisions)
    {
        Vec3 pointOnSphere1 = ClosestPointOnSphere(sphere1Pos, radius1, sphere2Pos);
        Vec3 pointOnSphere2 = ClosestPointOnSphere(sphere2Pos, radius2, sphere1Pos);
        Point::AddWorldPoint(Point(pointOnSphere1, Color::red, 4));
        Point::AddWorldPoint(Point(pointOnSphere2, Color::red, 4));
        Line::AddWorldLine(Line(pointOnSphere1, pointOnSphere2));
    }

    return collisionInfo.colliding;
}

bool SphereCubeColliding(SphereCollider& sphere, BoxCollider& cube, CollisionInfo& collisionInfo, bool resolve = true)
{
    collisionInfo.colliding = false;
    float radius = sphere.Radius();
    Vec3 sphereCenter = sphere.Position();
    Vec3 sphereCenter_cubeCoords = cube.TRInverse() * sphereCenter;
    Vec3 min = cube.Root().LocalScale4x4() * cube.mesh->bounds->min;
    Vec3 max = cube.Root().LocalScale4x4() * cube.mesh->bounds->max;

    Vec3 closestPointOnCube_cubeCoords;
    closestPointOnCube_cubeCoords.x = Clamp(sphereCenter_cubeCoords.x, min.x, max.x);
    closestPointOnCube_cubeCoords.y = Clamp(sphereCenter_cubeCoords.y, min.y, max.y);
    closestPointOnCube_cubeCoords.z = Clamp(sphereCenter_cubeCoords.z, min.z, max.z);

    Vec3 disp_cubeCoords = closestPointOnCube_cubeCoords - sphereCenter_cubeCoords;
    if (disp_cubeCoords.SqrMagnitude() < radius * radius)
    {
        collisionInfo.colliding = true;
        Vec3 closestOnCube = cube.TR() * closestPointOnCube_cubeCoords;
        Vec3 closestOnSphere = ClosestPointOnSphere(sphereCenter, radius, closestOnCube);
        Vec3 offset = closestOnSphere - closestOnCube;
        collisionInfo.lineOfImpact = offset;
        collisionInfo.pointOfContact = (closestOnCube + closestOnSphere) * 0.5; // check this
        if (resolve)
        {
            ResolveCollision(sphere, cube, offset);
        }
    }

    return collisionInfo.colliding;
}

// Oriented Bounding Box (OBB) with Separating Axis Theorem (SAT) algorithm
bool OBBSATColliding(BoxCollider& box1, BoxCollider& box2, BoxCollisionInfo& collisionInfo, bool resolve = true)
{
    bool gap = true;

    collisionInfo = BoxCollisionInfo();

    List<Vec3> physObj1Verts = box1.WorldVertices();
    List<Vec3> physObj2Verts = box2.WorldVertices();
    List<Vec3> physObj1Normals = List<Vec3>{ box1.Root().localRotation * Direction::right, box1.Root().localRotation * Direction::up, box1.Root().localRotation * Direction::forward };// mesh1.WorldXYZNormals();
    List<Vec3> physObj2Normals = List<Vec3>{ box2.Root().localRotation * Direction::right, box2.Root().localRotation * Direction::up, box2.Root().localRotation * Direction::forward }; //mesh2.WorldXYZNormals();

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
            gap = !((rangeA.max >= rangeB.min && rangeB.max >= rangeA.min));// || (mesh1Range.max < mesh2Range.min && mesh2Range.max < mesh1Range.min));
            if (gap) {
                collisionInfo.colliding = false;
                return false;
            }

            //Compare and cache minimum projection distance and axis for later use if needed for collision resolution.
            float potentialMinOverlap = 0;
            if (rangeA.max > rangeB.max) {
                potentialMinOverlap = rangeB.max - rangeA.min;
                axis *= -1.0;// Reverse push direction since object B is behind object A and we will always push A backwards and B forwards.
            }
            else {
                potentialMinOverlap = rangeA.max - rangeB.min;
            }

            if (i == 0)
            {
                collisionInfo.minOverlap = potentialMinOverlap;
                collisionInfo.minOverlapAxis = axis;
                collisionInfo.lineOfImpact = axis;
            }
            else if (potentialMinOverlap < collisionInfo.minOverlap)
            {
                collisionInfo.minOverlap = potentialMinOverlap;
                collisionInfo.minOverlapAxis = axis;
                collisionInfo.lineOfImpact = axis;
            }
        }
    }

    // Step 3: Must continue searching for possible 3D Edge-Edge collision
    if (!gap)
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
                    continue;
                }

                // Search for possible 3D Edge-Edge collision
                Vec3 axis = CrossProduct(nA, nB);
                Range rangeA = ProjectVertsOntoAxis(physObj1Verts.data(), physObj1Verts.size(), axis);
                Range rangeB = ProjectVertsOntoAxis(physObj2Verts.data(), physObj2Verts.size(), axis);
                gap = !((rangeA.max >= rangeB.min && rangeB.max >= rangeA.min));// || (mesh1Range.max < mesh2Range.min && mesh2Range.max < mesh1Range.min));
                if (gap) {
                    collisionInfo.colliding = false;
                    return false;
                }
                //Does not overwrite minOverlapAxis with these axes. Allows boxes to always push from normals.
            }
        }
    }

    collisionInfo.colliding = !gap;

    if (collisionInfo.colliding)
    {
        if (resolve)
        {
            Vec3 offset = collisionInfo.minOverlapAxis * collisionInfo.minOverlap;
            ResolveCollision(box1, box2, offset);
        }
    }

    return collisionInfo.colliding;
}

// Collision influence precedence: Static = 3, Kinematic = 2, Dynamic = 1
void ResolveCollision(Collider& colliderA, Collider& colliderB, Vec3& penDepth)
{
    static float allowablePenDepth = 0.0055;
    //static float sqrAllowablePenDepth = allowablePenDepth * allowablePenDepth;

    if (penDepth.Magnitude() <= allowablePenDepth)
    {
        return;
    }

    if (colliderA.isStatic && colliderB.isStatic)
    {
        return;
    }

    if (&colliderA.Root() != &colliderB.Root())
    {
        penDepth.x - allowablePenDepth;
        penDepth.y - allowablePenDepth;
        penDepth.z - allowablePenDepth;

        if (colliderA.isStatic)
        {
            colliderB.Root().localPosition += penDepth * 1.0001;
        }
        else if (colliderB.isStatic)
        {
            colliderA.Root().localPosition -= penDepth * 1.0001;
        }
        else if (colliderA.object->isKinematic && !colliderA.isStatic)
        {
            colliderB.Root().localPosition += penDepth * 1.0001;
        }
        else if (colliderB.object->isKinematic && !colliderB.isStatic)
        {
            colliderA.Root().localPosition -= penDepth * 1.0001;
        }
        else
        {
            penDepth *= 0.50001;//0.51 0.501
            colliderA.Root().localPosition -= penDepth;
            colliderB.Root().localPosition += penDepth;
        }
    }
}

void OnCollision(Collider& colliderA, Collider& colliderB, Vec3& lineOfImpact)
{
    colliderA.OnCollision(&colliderB);
    colliderB.OnCollision(&colliderA);

    if (Physics::dynamics)
    {
        if (&colliderA.Root() != &colliderB.Root())
        {
            if (colliderA.isStatic || colliderA.object->isKinematic)
            {
                CalculateStaticCollision(lineOfImpact, colliderB.object->velocity, colliderB.coefficientRestitution);
            }
            else if (colliderB.isStatic || colliderB.object->isKinematic)
            {
                CalculateStaticCollision(lineOfImpact, colliderA.object->velocity, colliderA.coefficientRestitution);
            }
            else
            {
                float e = colliderA.coefficientRestitution < colliderB.coefficientRestitution ? colliderA.coefficientRestitution : colliderB.coefficientRestitution;//((colliderA.coefficientRestitution + colliderB.coefficientRestitution) / 2.0)

                CalculateCollision(
                    lineOfImpact,
                    colliderA.object->mass,
                    colliderB.object->mass,
                    colliderA.object->velocity,
                    colliderB.object->velocity,
                    e
                );
            }
        }
    }
};

void DetectCollisions()
{
    // How nested loop algorithm works: 
    // Gets colliders A, B, C, D, E...
    // Compare A:B, A:C, A:D, A:E
    // Compare B:C, B:D, B:E
    // Compare C:D, C:E
    // Compare D:E

    // ****************************************************************************************************
    //                                      BOX-BOX COLLISIONS
    // ****************************************************************************************************
    for (size_t i = 0; i < ManagedObjectPool<BoxCollider>::count; i++)
    {
        // exit if this is the last Collider
        if ((i + 1) >= ManagedObjectPool<BoxCollider>::count)
        {
            break;
        }

        // Current Collider
        BoxCollider* box1 = ManagedObjectPool<BoxCollider>::objects[i];

        for (size_t j = i + 1; j < ManagedObjectPool<BoxCollider>::count; j++)
        {
            // Next Collider
            BoxCollider* box2 = ManagedObjectPool<BoxCollider>::objects[j];

            BoxCollisionInfo collisionInfo;
            bool resolveIfNotTrigger = !(box1->isTrigger || box2->isTrigger);
            if (OBBSATColliding(*box1, *box2, collisionInfo, resolveIfNotTrigger))
            {
                OnCollision(*box1, *box2, collisionInfo.minOverlapAxis);
            }
        }
    }

    // ****************************************************************************************************
    //                                  SPHERE-SPHERE COLLISIONS
    // ****************************************************************************************************

    for (size_t i = 0; i < ManagedObjectPool<SphereCollider>::count; i++)
    {
        // Current Collider
        SphereCollider* sphere1 = ManagedObjectPool<SphereCollider>::objects[i];

        // SPHERE-SPHERE COLLISIONS
        for (size_t j = i + 1; j < ManagedObjectPool<SphereCollider>::count; j++)
        {
            // Next Collider
            SphereCollider* sphere2 = ManagedObjectPool<SphereCollider>::objects[j];

            CollisionInfo collisionInfo;
            bool resolveIfNotTrigger = !(sphere1->isTrigger || sphere2->isTrigger);
            if (SpheresColliding(*sphere1, *sphere2, collisionInfo, resolveIfNotTrigger))
            {
                OnCollision(*sphere1, *sphere2, collisionInfo.lineOfImpact);
            }
        }

        // ****************************************************************************************************
        //                                      SPHERE-PLANE COLLISIONS
        // ****************************************************************************************************

        for (size_t ii = 0; ii < ManagedObjectPool<PlaneCollider>::count; ii++)
        {
            // Next Collider
            PlaneCollider* plane = ManagedObjectPool<PlaneCollider>::objects[ii];

            CollisionInfo collisionInfo;
            bool resolveIfNotTrigger = !(sphere1->isTrigger || plane->isTrigger);
            if (SpherePlaneColliding(*sphere1, *plane, collisionInfo, resolveIfNotTrigger))
            {
                OnCollision(*sphere1, *plane, collisionInfo.lineOfImpact);
            }
        }
    }

    // ****************************************************************************************************
    //                                      SPHERE-BOX COLLISIONS
    // ****************************************************************************************************
    // 
    // Checks every cube against every sphere
    for (size_t i = 0; i < ManagedObjectPool<BoxCollider>::objects.size(); i++)
    {
        // Current Collider
        BoxCollider* box = ManagedObjectPool<BoxCollider>::objects[i];
        for (size_t j = 0; j < ManagedObjectPool<SphereCollider>::objects.size(); j++)
        {
            // Sphere Collider
            SphereCollider* sphere = ManagedObjectPool<SphereCollider>::objects[j];

            CollisionInfo collisionInfo;
            bool resolveIfNotTrigger = !(sphere->isTrigger || box->isTrigger);
            if (SphereCubeColliding(*sphere, *box, collisionInfo, resolveIfNotTrigger))
            {
                OnCollision(*sphere, *box, collisionInfo.lineOfImpact);
            }
        }
    }
}

void DetectCollisionsOctTree()
{
    // ****************************************************************************************************
    //                                          BOX-BOX COLLISIONS
    // ****************************************************************************************************

    OctTree<BoxCollider>::Update();

    for (size_t i = 0; i < ManagedObjectPool<BoxCollider>::count; i++)
    {
        // Current Collider
        BoxCollider* box1 = ManagedObjectPool<BoxCollider>::objects[i];
        box1->flagged = true;

        // Search for nearby colliders
        auto volume = Cube(box1->TRS() * box1->mesh->bounds->min, box1->TRS() * box1->mesh->bounds->max);
        auto closestBoxes = OctTree<BoxCollider>::Search(volume, [&](Collider* collider) { return !collider->flagged; });

        for (size_t j = 0; j < closestBoxes->size(); j++)
        {
            // Next Collider
            BoxCollider* box2 = (*closestBoxes)[j];

            BoxCollisionInfo collisionInfo;
            bool resolveIfNotTrigger = !(box1->isTrigger || box2->isTrigger);
            if (OBBSATColliding(*box1, *box2, collisionInfo, resolveIfNotTrigger))
            {
                OnCollision(*box1, *box2, collisionInfo.lineOfImpact);//collisionInfo.minOverlapAxis);
            }
        }
    }

    for (size_t i = 0; i < ManagedObjectPool<BoxCollider>::objects.size(); i++)
    {
        ManagedObjectPool<BoxCollider>::objects[i]->flagged = false;
    }

    // ****************************************************************************************************
    //                                  SPHERE-SPHERE COLLISIONS
    // ****************************************************************************************************

    OctTree<SphereCollider>::Update();

    for (size_t i = 0; i < ManagedObjectPool<SphereCollider>::objects.size(); i++)
    {
        // Current Collider
        SphereCollider* sphere1 = ManagedObjectPool<SphereCollider>::objects[i];
        sphere1->flagged = true;

        // Search for nearby colliders
        auto volume = Cube(sphere1->TRS() * sphere1->mesh->bounds->min, sphere1->TRS() * sphere1->mesh->bounds->max);
        auto closestSpheres = OctTree<SphereCollider>::Search(volume, [&](Collider* collider) { return !collider->flagged; });

        for (size_t j = 0; j < closestSpheres->size(); j++)
        {
            // Next Collider
            SphereCollider* sphere2 = (*closestSpheres)[j];

            CollisionInfo collisionInfo;
            bool resolveIfNotTrigger = !(sphere1->isTrigger || sphere2->isTrigger);
            if (SpheresColliding(*sphere1, *sphere2, collisionInfo, resolveIfNotTrigger))
            {
                OnCollision(*sphere1, *sphere2, collisionInfo.lineOfImpact);
            }
        }
    }

    for (size_t i = 0; i < ManagedObjectPool<SphereCollider>::objects.size(); i++)
    {
        ManagedObjectPool<SphereCollider>::objects[i]->flagged = false;
    }

    // ****************************************************************************************************
    //                                      SPHERE-BOX COLLISIONS
    // ****************************************************************************************************
    // 
    // Check if more cubes than boxes before determining algorithm. If there are more cubes than spheres
    // it makes more sense to search for the closest cubes for each sphere than the other way around.
    if (ManagedObjectPool<SphereCollider>::count < ManagedObjectPool<BoxCollider>::count)
    {
        for (size_t i = 0; i < ManagedObjectPool<SphereCollider>::objects.size(); i++)
        {
            // Current Collider
            SphereCollider* sphere = ManagedObjectPool<SphereCollider>::objects[i];
            sphere->flagged = true;

            // Search for nearby colliders
            auto volume = Cube(sphere->TRS() * sphere->mesh->bounds->min, sphere->TRS() * sphere->mesh->bounds->max);
            auto closestObjects = OctTree<BoxCollider>::Search(volume, [&](Collider* collider) { return !collider->flagged; });

            for (size_t j = 0; j < closestObjects->size(); j++)
            {
                // Next Collider
                BoxCollider* box = (*closestObjects)[j];

                CollisionInfo collisionInfo;
                bool resolveIfNotTrigger = !(sphere->isTrigger || box->isTrigger);
                if (SphereCubeColliding(*sphere, *box, collisionInfo, resolveIfNotTrigger))
                {
                    OnCollision(*sphere, *box, collisionInfo.lineOfImpact);
                }
            }
        }

        for (size_t i = 0; i < ManagedObjectPool<SphereCollider>::objects.size(); i++)
        {
            ManagedObjectPool<SphereCollider>::objects[i]->flagged = false;
        }
    }
    // Algorithm when there are more spheres than cubes
    else
    {
        for (size_t i = 0; i < ManagedObjectPool<BoxCollider>::objects.size(); i++)
        {
            // Current Collider
            BoxCollider* box = ManagedObjectPool<BoxCollider>::objects[i];
            box->flagged = true;

            // Search for nearby colliders
            auto volume = Cube(box->TRS() * box->mesh->bounds->min, box->TRS() * box->mesh->bounds->max);
            auto closestObjects = OctTree<SphereCollider>::Search(volume, [&](Collider* collider) { return !collider->flagged; });

            for (size_t j = 0; j < closestObjects->size(); j++)
            {
                // Next Collider
                SphereCollider* sphere = (*closestObjects)[j];

                CollisionInfo collisionInfo;
                bool resolveIfNotTrigger = !(sphere->isTrigger || box->isTrigger);
                if (SphereCubeColliding(*sphere, *box, collisionInfo, resolveIfNotTrigger))
                {
                    OnCollision(*sphere, *box, collisionInfo.lineOfImpact);
                }
            }
        }

        for (size_t i = 0; i < ManagedObjectPool<BoxCollider>::objects.size(); i++)
        {
            ManagedObjectPool<BoxCollider>::objects[i]->flagged = false;
        }
    }
}

class Ray
{
protected:
    Transform transform = Transform();
    Vec3 endPosition;
    Vec3 direction;
public:

    float distance = 1;

    Ray(Vec3 start, Vec3 end)
    {
        Set(start, end);
    }

    Ray(Vec3 start, Vec3 dir, float dist)
    {
        Set(start, start + dir * dist);
    }

    void Set(Vec3 start, Vec3 end)
    {
        Vec3 disp = (end - start);
        distance = disp.Magnitude();
        direction = disp.Normalized();
        transform.localRotation = OrthogonalMatrixLookAt(direction);
        transform.localPosition = start;
        endPosition = end;
    }

    Vec3 StartPosition()
    {
        return transform.Position();
    }

    Vec3 EndPosition()
    {
        return endPosition;
    }

    Vec3 Direction()
    {
        return direction;
    }

    Matrix4x4 WorldToRaySpaceMatrix()
    {
        return transform.TRInverse();
    }
};

template <typename T>
struct RaycastInfo
{
    T* objectHit = nullptr;
    Triangle* triangleHit = nullptr;
    Triangle triangleHit_w = Triangle();
    Vec3 contactPoint = Vec3::zero;
    RaycastInfo() {};
    RaycastInfo(T* objHit, Triangle* triHit, Vec3& contactPoint, Triangle& worldSpaceTri)
    {
        this->objectHit = objHit;
        this->triangleHit = triHit;
        this->contactPoint = contactPoint;
        this->triangleHit_w = worldSpaceTri;
    }
};

template <typename T>
bool Raycast(Ray& ray, RaycastInfo<T>& raycastInfo, const std::function<void(RaycastInfo<T>&)>& callback = NULL)
{
    Vec3 from = ray.StartPosition();
    Vec3 to = ray.EndPosition();

    float closestSqrDistHit = ray.distance * ray.distance;
    for (size_t i = 0; i < ManagedObjectPool<T>::objects.size(); i++)
    {
        auto obj = ManagedObjectPool<T>::objects[i];

        List<Triangle>* triangles = obj->MapVertsToTriangles();
        for (size_t j = 0; j < triangles->size(); j++)
        {
            Triangle worldSpaceTri = (*triangles)[j];
            for (size_t k = 0; k < 3; k++) {
                worldSpaceTri.verts[k] = obj->TRS() * worldSpaceTri.verts[k];
            }
            //------------------Ray casting (World & Ray Space)--------------------------
            Vec3 pointOfIntersection;
            if (LinePlaneIntersecting(from, to, worldSpaceTri, &pointOfIntersection))
            {
                // If not behind raycast
                if (DotProduct(pointOfIntersection - from, ray.Direction()) >= 0)
                {
                    if (Graphics::debugRaycasting) {
                        Line::AddWorldLine(Line(from, to, Color::red));
                    }
                    Matrix4x4 worldToRaySpaceMatrix = ray.WorldToRaySpaceMatrix();
                    Vec3 pointOfIntersection_v = worldToRaySpaceMatrix * pointOfIntersection;
                    Triangle* viewSpaceTri = &((*triangles)[j]);
                    for (size_t k = 0; k < 3; k++) {
                        viewSpaceTri->verts[k] = worldToRaySpaceMatrix * worldSpaceTri.verts[k];
                    }
                    if (PointInsideTriangle(pointOfIntersection_v, viewSpaceTri->verts))
                    {
                        // Check if within range
                        float sqrDist = (pointOfIntersection - from).SqrMagnitude();
                        if (sqrDist <= closestSqrDistHit)
                        {
                            closestSqrDistHit = sqrDist;

                            raycastInfo.objectHit = obj;
                            raycastInfo.contactPoint = pointOfIntersection;
                            raycastInfo.triangleHit = &(*triangles)[j];
                            raycastInfo.triangleHit_w = worldSpaceTri;

                            if (callback) {
                                callback(raycastInfo);
                            }

                            // ---------- Debugging -----------
                            if (Physics::raycastDebugging)
                            {
                                Point::AddWorldPoint(Point(pointOfIntersection_v, Color::red, 5));
                                // Reflect
                                Vec3 n = worldSpaceTri.Normal();
                                Vec3 v = (pointOfIntersection - from);
                                Vec3 reflection = Reflect(v, n);
                                Line::AddWorldLine(Line(pointOfIntersection, pointOfIntersection + reflection, Color::red));

                                // Project
                                Vec3 vecPlane = ProjectOnPlane(v, n);
                                Line::AddWorldLine(Line(pointOfIntersection, pointOfIntersection + vecPlane, Color::pink));
                            }
                        }
                    }
                }
            }
        }
    }
    return raycastInfo.objectHit != nullptr;
}

template <typename T>
bool Raycast(Vec3 from, Vec3 to, RaycastInfo<T>& raycastInfo, const std::function<void(RaycastInfo<T>&)>& callback = NULL)
{
    Ray ray = Ray(from, to);
    return Raycast(ray, raycastInfo);
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
        cam->localPosition += moveDir * accel * _engineDeltaTime;
    }
    else
    {
        velocity += moveDir * accel * _engineDeltaTime;
        /*if (Physics::gravity) {
            velocity += gravity * _engineDeltaTime;
        }
        else if*/
        if (dampenersActive) {
            velocity += velocity * decel * _engineDeltaTime;
        }
        if (velocity.SqrMagnitude() < 0.0001) {
            velocity = Vec3::zero;
        }
        cam->localPosition += velocity * _engineDeltaTime;
    }

    if (Physics::dynamics)
    {
        for (size_t i = 0; i < ManagedObjectPool<PhysicsObject>::count; i++)
        {
            PhysicsObject* obj = ManagedObjectPool<PhysicsObject>::objects[i];
            if (!obj->collider->isStatic && !obj->isKinematic)
            {
                if (Physics::gravity) {
                    obj->velocity += gravity * deltaTime;
                }
                obj->localPosition += obj->velocity * deltaTime;
                obj->localRotation = Matrix3x3::RotAxisAngle(obj->angVelAxis, obj->angVelSpeed * deltaTime) * obj->localRotation;
            }
        }
    }

    if (Physics::collisionDetection)
    {
        if (Physics::octTree)
        {
            DetectCollisionsOctTree();
        }
        else {
            DetectCollisions();
        }
    }

    if (Physics::raycasting)
    {
        Ray ray1 = Ray(Camera::cameras[1]->Position(), Camera::cameras[1]->Position() + Camera::cameras[1]->Forward() * 50);
        RaycastInfo<Mesh> info;
        if (Raycast(ray1, info))
        {
            //cout << "RAYCAST HIT" << '\n';
            Line::AddWorldLine(Line(ray1.StartPosition(), ray1.EndPosition(), Color::green, 3));
            Point::AddWorldPoint(Point(info.contactPoint, Color::green, 7));
            //info.objectHit->SetColor(Color::purple);
            info.triangleHit->color = Color::purple;//Color::Random();
        }
        /*
        Ray ray2 = Ray(Camera::cameras[2]->Position(), Camera::cameras[2]->Forward(), 50);
        RaycastInfo<Collider> info2;
        if (Raycast<Collider>(ray2, info2))
        {
            //cout << "RAYCAST HIT" << '\n';
            Line::AddWorldLine(Line(ray2.StartPosition(), ray2.EndPosition(), Color::green, 3));
            Point::AddWorldPoint(Point(info2.contactPoint, Color::green, 7));
            info2.objectHit->object->mesh->SetColor(Color::red);
        }*/
    }
}
#endif