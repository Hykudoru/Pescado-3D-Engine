#pragma once
#include <GLFW/glfw3.h>
#include <math.h>
#include <Matrix.h>
#include <Graphics.h>
#include <Utility.h>
#include <Functional>
using namespace std;


/*TO-DO
* 
* // OPTIMIZATIONS
* Revisit sphere-plane collisions (possible performance optimization)
* Box collisions not quite right
* 
* // ISSUES
* Fix Physics Object collider parenting issue
* 
* // FUTURE (POSSIBLE BIG CHANGE)
* World Partitioning
* 
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
    static bool collisionDetection;
    static bool dynamics;
    static bool raycasting;
    static bool raycastDebugging;
    static bool gravity;
};
bool Physics::collisionDetection = true;
bool Physics::dynamics = true;
bool Physics::raycasting = false;
bool Physics::raycastDebugging = false;
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

class Component : public Transform
{
public:
    PhysicsObject* object;
};

class RigidBody
{
public:
    Vec3 centerOfMass = Vec3::zero;
    float mass = 1;
    bool isKinematic = false;
    Vec3 velocity = Vec3::zero;
    Vec3 angularVelocity = Vec3::zero;
};

class Collider : public Component, public  ManagedObjectPool<Collider>
{
public:
    Mesh* mesh;
    bool isStatic = false;
    bool isTrigger = false;
    float coefficientRestitution = 1.0;

    Collider(bool isStatic = false, bool isTrigger = false): ManagedObjectPool<Collider>(this)
    {
        this->isStatic = isStatic;
        this->isTrigger = isTrigger;
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
};

class BoxCollider : public Collider, public ManagedObjectPool<BoxCollider>
{
public:
    BoxCollider(bool isStatic = false, bool isTrigger = false) : Collider(isStatic, isTrigger), ManagedObjectPool<BoxCollider>(this)
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
    
    SphereCollider(bool isStatic = false, bool isTrigger = false) : Collider(isStatic, isTrigger), ManagedObjectPool<SphereCollider>(this)
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
    PlaneCollider(Vec3 normal, bool isStatic = false, bool isTrigger = false) : Collider(isStatic, isTrigger), ManagedObjectPool<PlaneCollider>(this)
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

    PhysicsObject(Mesh* mesh, Collider* collider) : ManagedObjectPool<PhysicsObject>(this)
    { 
        SetCollider(collider);
        SetMesh(mesh);
    }

    PhysicsObject(float scale, Vec3 position, Matrix3x3 rotation, Mesh* mesh, Collider* collider) : ManagedObjectPool<PhysicsObject>(this)
    {
        //this->scale = Vec3(scale, scale, scale);
        this->localPosition = position;
        this->localRotation = rotation;
        this->localScale = Vec3(scale, scale, scale);
        SetCollider(collider);
        SetMesh(mesh);
    }

    virtual ~PhysicsObject()
    {
        delete collider;
        delete mesh;
    }

    void SetCollider(Collider* collider)
    {
        if (collider)
        {
            delete this->collider;
            this->collider = collider;
            this->collider->object = this;
            this->collider->SetParent(this, false);
        }
    }

    void SetMesh(Mesh* mesh)
    {
        if (mesh)
        {
            delete this->mesh;
            this->mesh = mesh;
            this->mesh->SetParent(collider, false);
        }
    }
};

struct CollisionInfo
{
    bool colliding = false;
};

struct BoxCollisionInfo : public CollisionInfo
{
    float minOverlap = 0;
    Vec3 minOverlapAxis = Vec3::zero;
};

struct SphereCollisionInfo : public CollisionInfo
{
    Vec3 pointOfContact = Vec3::zero;
    Vec3 lineOfImpact = Vec3::zero;
};

void CalculateCollision(Vec3 lineOfImpact, float& m1, float& m2, Vec3& v1, Vec3& v2, float e = 1.0)
{
    /* Elastic collision (conserves both momentum and kinetic energy)
    Conservation Momentum: m1*v1 + m2*v2 = m1*v1' + m2*v2'
    Conservation Kinetic Energy: v1 + v1' = v2 + v2'
    Coefficient of Restitution: e = v2'-v1' / v1-v2
        e = 1 Perfectly elastic
        0 < e < 1 inelastic
        e = 0 Perfectly inelastic
    */
    lineOfImpact.Normalize();
    Vec3 v1LineOfImpact = lineOfImpact * DotProduct(v1, lineOfImpact);
    Vec3 v2LineOfImpact = lineOfImpact * DotProduct(v2, lineOfImpact);
    Vec3 v1LineOfImpactFinal = (v1LineOfImpact * m1 + v2LineOfImpact * m2 * 2.0 - v1LineOfImpact * m2) * (1.0 / (m1 + m2));
    Vec3 v2LineOfImpactFinal = ((v1LineOfImpact - v2LineOfImpact) * e) + v1LineOfImpactFinal;// e(v1-v2)+v1' = v2'
    Vec3 v1PerpendicularFinal = (v1 - v1LineOfImpact);//Perpendicular Velocity is the same before and after impact
    Vec3 v2PerpendicularFinal = (v2 - v2LineOfImpact);//Perpendicular Velocity is the same before and after impact
    Vec3 v1Final = v1LineOfImpactFinal + v1PerpendicularFinal;
    Vec3 v2Final = v2LineOfImpactFinal + v2PerpendicularFinal;

    v1 = v1Final;
    v2 = v2Final;
}

void CalculateStaticCollision(Vec3 lineOfImpact, Vec3& v1, float e = 1.0)
{
    Vec3 v1LineOfImpact = lineOfImpact * DotProduct(v1, lineOfImpact);
    Vec3 v1PerpendicularFinal = (v1 - v1LineOfImpact);//Perpendicular Velocity is the same before and after impact
    v1 = (v1LineOfImpact * -e) + v1PerpendicularFinal;
}

bool SpherePlaneColliding(SphereCollider& sphere, PlaneCollider& plane, SphereCollisionInfo& collisionInfo, bool resolve = true)
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
        if (resolve)
        {
            Vec3 pointOnSphere = ClosestPointOnSphere(sphereCenter, radius, closestPointOnPlane);
            Vec3 offset = pointOnSphere - closestPointOnPlane;//overlapping
            sphere.root->localPosition -= offset;
            collisionInfo.lineOfImpact = normal * -1.0;
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

bool SpheresColliding(SphereCollider& sphere1, SphereCollider& sphere2, SphereCollisionInfo& collisionInfo, bool resolve = true)
{
    float radius1 = sphere1.Radius();
    float radius2 = sphere2.Radius();
    Vec3 sphere1Pos = sphere1.Position();
    Vec3 sphere2Pos = sphere2.Position();

    collisionInfo.colliding = (sphere2Pos - sphere1Pos).SqrMagnitude() < (radius1 + radius2) * (radius1 + radius2);
    if (collisionInfo.colliding)
    {
        Vec3 pointOnSphere1 = ClosestPointOnSphere(sphere1Pos, radius1, sphere2Pos);
        Vec3 pointOnSphere2 = ClosestPointOnSphere(sphere2Pos, radius2, sphere1Pos);
        Vec3 offset = (pointOnSphere1 - pointOnSphere2);
        collisionInfo.lineOfImpact = offset;
        if (resolve) {
            offset *= 0.5;
            sphere1.root->localPosition -= offset;
            sphere2.root->localPosition += offset;
            collisionInfo.pointOfContact = (pointOnSphere1 + pointOnSphere2) * 0.5;
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

// Oriented Bounding Box (OBB) with Separating Axis Theorem (SAT) algorithm
bool OBBSATColliding(BoxCollider& box1, BoxCollider& box2, BoxCollisionInfo& collisionInfo, bool resolve = true)
{
    bool gap = true;

    collisionInfo = BoxCollisionInfo();

    if (box1.isStatic && box2.isStatic)
    {
        return false;
    }

    List<Vec3> physObj1Verts = box1.WorldVertices();
    List<Vec3> physObj2Verts = box2.WorldVertices();
    List<Vec3> physObj1Normals = List<Vec3>{ box1.root->localRotation * Direction::right, box1.root->localRotation * Direction::up, box1.root->localRotation * Direction::forward };// mesh1.WorldXYZNormals();
    List<Vec3> physObj2Normals = List<Vec3>{ box2.root->localRotation * Direction::right, box2.root->localRotation * Direction::up, box2.root->localRotation * Direction::forward }; //mesh2.WorldXYZNormals();

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
            }
            else if (potentialMinOverlap < collisionInfo.minOverlap)
            {
                collisionInfo.minOverlap = potentialMinOverlap;
                collisionInfo.minOverlapAxis = axis;
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
                gap = !((rangeA.max >= rangeB.min && rangeB.max >= rangeA.min));// || (mesh1Range.max < mesh2Range.min && mesh2Range.max < mesh1Range.min));
                if (gap) {
                    collisionInfo.colliding = false;
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

    collisionInfo.colliding = !gap;

    if (collisionInfo.colliding && resolve)
    {
        Vec3 offset = collisionInfo.minOverlapAxis * collisionInfo.minOverlap;

        bool neitherStatic = !box1.isStatic && !box2.isStatic;
        if (neitherStatic)
        {
            offset *= 0.5;
            box1.root->localPosition -= (offset * 1.01);
            box2.root->localPosition += (offset * 1.01);
        }
        //Only one is movable at this stage
        else if (box1.isStatic) {
            box2.root->localPosition += offset;
        }
        else {
            box1.root->localPosition -= offset;
        }
    }

    return collisionInfo.colliding;
}

template <typename T>
class TreeNode : public CubeMesh
{
protected:
    Vec3 min_w;
    Vec3 max_w;
public:
    int level = 0;
    int maxDepth = 16;
    int maxCapacity = 2;
    int maxChildren = 8;
    TreeNode<T>* root = nullptr;
    TreeNode<T>* parent = nullptr;
    List<T*> contained = List<T*>();
    List<TreeNode<T>*>* children = nullptr;

    TreeNode(TreeNode<T>* parent = nullptr)
    {
        if (!parent)
        {
            this->root = this;
            this->localScale = Vec3(100000, 100000, 100000);
        }
        else
        {
            this->parent = parent;
            this->root = parent->root;
            this->level = parent->level + 1;
            this->localScale = (parent->localScale * .5);
            if (this->level > 1)
            {
                this->SetColor(this->parent->color);
            }
        }

        this->SetVisibility(false);
    }

    virtual ~TreeNode<T>()
    {
        if (children)
        {
            for (size_t i = 0; i < children->size(); i++)
            {
                delete (*children)[i];
            }

            delete children;
        }
    }

    void Subdivide()
    {
        if (!children && level < maxDepth)
        {
            children = new List<TreeNode<T>*>();

            for (int width = -1; width <= 1; width += 2)
            {
                for (int height = -1; height <= 1; height += 2)
                {
                    for (int depth = -1; depth <= 1; depth += 2)
                    {
                        auto newChild = new TreeNode<T>(this);
                        newChild->localPosition = Position() + Direction::right * width * .5 * newChild->localScale.x + Direction::up * height *.5 * newChild->localScale.y + Direction::forward * depth * .5 * newChild->localScale.z;
                        newChild->bounds->CreateBounds(newChild);
                        newChild->min_w = newChild->TRS() * newChild->bounds->min;
                        newChild->max_w = newChild->TRS() * newChild->bounds->max;

                        children->emplace_back(newChild);
                    }
                }
            }
        }
    }

    bool OverlappingPoint(Vec3& point)
    {
        if (point.x >= min_w.x && point.x <= max_w.x
            && point.y >= min_w.y && point.y <= max_w.y
            && point.z >= min_w.z && point.z <= max_w.z)
        {
            return true;
        }
        
        return false;
    }

    bool Overlapping(T* obj)
    {
        //if atleast containing position, check if completely overlapping bounds.
        Vec3 pos = obj->Position();
        if (OverlappingPoint(pos))
        {
            Mesh* mesh = dynamic_cast<Mesh*>(obj);
            if (mesh)
            {
                auto verts = mesh->bounds->WorldVertices();
                for (size_t i = 0; i < 8; i++)
                {
                    if (!OverlappingPoint(verts[i]))
                    {
                        return false;
                    }
                }
                return true;
            }

            PhysicsObject* physObj = dynamic_cast<PhysicsObject*>(obj);
            if (physObj)
            {
                auto verts = physObj->mesh->bounds->WorldVertices();
                for (size_t i = 0; i < 8; i++)
                {
                    if (!OverlappingPoint(verts[i]))
                    {
                        return false;
                    }
                }

                return true;
            }
        }

        return false;
    }
    
    TreeNode<T>* Query(Vec3& pos, List<T*>& list)
    {
        if (this->OverlappingPoint(pos))
        {
            cout << " level: " << this->level << endl;
            this->Extract(list);
            
            if (!this->children)
            {
                return this;
            }
            else 
            {
                for (size_t i = 0; i < this->children->size(); i++)
                {
                    auto node = (*children)[i];
                    node = node->Query(pos, list);
                    if (node != nullptr) {
                        return node;
                    }
                }
            }
        }

        return nullptr;
    }

    TreeNode<T>* Insert(T* obj)
    {
        // Prevent box from being inserted into itself.
        if (obj == (T*)this)
        {
            return nullptr;
        }

        if (Overlapping(obj))
        {
            if (contained.size() < maxCapacity)
            {
                contained.emplace_back(obj);
                return this;
            }
            else 
            {
                if (!children && level < maxDepth)
                {
                    Subdivide();
                }

                if (children)
                {
                    for (size_t i = 0; i < children->size(); i++)
                    {
                        TreeNode<T>* node = (*children)[i];
                        node = node->Insert(obj);
                        if (node) {
                            return node;
                        }
                    }
                }
            }
        }

        return nullptr;
    }

    void Extract(List<T*>& list)
    {
        for (size_t i = 0; i < this->contained.size(); i++)
        {
            list.emplace_back(this->contained.at(i));
        }
    }

    void Draw()
    {
        if (Graphics::debugTree && this->level > 0)
        {
            this->SetVisibility(true);
        }
        else {
            this->SetVisibility(false);
        }

        this->bounds->color = color;

        //Point::AddWorldPoint(Point(this->Position(), this->color, 15));
        //Point::AddWorldPoint(Point(min_w, Color::blue, 10));
        //Point::AddWorldPoint(Point(max_w, Color::blue, 10));
        //Line::AddWorldLine(Line(min_w, max_w, Color::pink, 5));

        for (size_t i = 0; i < this->contained.size(); i++)
        {
            auto obj = this->contained.at(i);
            Point::AddWorldPoint(Point(obj->Position(), this->color, 8));
        }

        if (this->children)
        {
            for (size_t i = 0; i < this->children->size(); i++)
            {
                auto child = (*this->children)[i]; 
                child->Draw();
            }
        }
    }
};

template <typename T>
class OctTree : public TreeNode<T>
{
public:
    static OctTree<T>* tree;

    OctTree() : TreeNode<T>()
    {
        this->Subdivide();
        
        (*this->children)[0]->SetColor(Color::red);
        (*this->children)[1]->SetColor(Color::orange);
        (*this->children)[2]->SetColor(Color::yellow);
        (*this->children)[3]->SetColor(Color::green);
        (*this->children)[4]->SetColor(Color::blue);
        (*this->children)[5]->SetColor(Color::purple);
        (*this->children)[6]->SetColor(Color::pink);
        (*this->children)[7]->SetColor(Color::turquoise);

        // Insert world objects
        for (size_t i = 0; i < ManagedObjectPool<T>::count; i++)
        {
            T* objTesting = ManagedObjectPool<T>::objects[i];

            bool inserted = false;
            for (size_t ii = 0; ii < this->children->size(); ii++)
            {
                auto child = (*this->children)[ii];
                auto node = child->Insert(objTesting);
                if (node) {
                    inserted = true;
                    break;
                }
            }

            if (!inserted) 
            {
                // Prevent box from being inserted into itself.
                if (objTesting != (T*)this)
                {
                    //objects to big or not encapsulated
                    this->contained.emplace_back(objTesting);
                }
            }
        }
    }

    static OctTree<T>* Tree()
    {
        if (!tree)
        {
            tree = new OctTree<T>();
        }

        return tree;
    }

    static void Update()
    {
        if (tree)
        {
            delete tree;
        }
        
        tree = new OctTree<T>();

        tree->Draw();
    }

    static List<T*> Search(Vec3& pos, std::function<void(T*)> func = NULL)
    {
        List<T*> list = List<T*>();
        Tree()->Extract(list);
        for (size_t i = 0; i < 8; i++)
        {
           auto node = (*Tree()->children)[i]->Query(pos, list);
           if (node) {
               break;
           }
        }

        if (func)
        {
            for (size_t i = 0; i < list.size(); i++)
            {
                func(list[i]);
            }
        }
        return list;
    }
};
template <typename T>
OctTree<T>* OctTree<T>::tree = nullptr;

void DetectCollisions()
{
    // How nested loop algorithm works: 
    // Gets colliders A, B, C, D, E...
    // Compare A:B, A:C, A:D, A:E
    // Compare B:C, B:D, B:E
    // Compare C:D, C:E
    // Compare D:E
    for (size_t i = 0; i < ManagedObjectPool<BoxCollider>::count; i++)
    {
        // exit if this is the last Collider
        if ((i + 1) >= ManagedObjectPool<BoxCollider>::count) {
            break;
        }

        // Current Collider
        BoxCollider* box1 = ManagedObjectPool<BoxCollider>::objects[i];

        for (size_t j = i + 1; j < ManagedObjectPool<BoxCollider>::count; j++)
        {
            // Next Collider
            BoxCollider* box2 = ManagedObjectPool<BoxCollider>::objects[j];

            if (box1->isStatic && box2->isStatic) {
                continue;
            }

            BoxCollisionInfo collisionInfo;
            bool resolveIfNotTrigger = !(box1->isTrigger || box2->isTrigger);
            if (OBBSATColliding(*box1, *box2, collisionInfo, resolveIfNotTrigger))
            {
                if (Physics::dynamics)
                {
                    if (box1->object->isKinematic || box2->object->isKinematic) {
                        continue;
                    }
                    /*
                    Although static objects themselves are not effected by momentum
                    transfers, their velocity variable may still be updating from new collisions.
                    Consequently, objects touching a static collider would be effected, so the
                    velocity is zeroed out to prevent this.
                    */
                    if (box1->isStatic)
                    {
                        box1->object->velocity = Vec3::zero;
                    }
                    else if (box2->isStatic)
                    {
                        box2->object->velocity = Vec3::zero;
                    }

                    CalculateCollision(
                        collisionInfo.minOverlapAxis,
                        box1->object->mass,
                        box2->object->mass,
                        box1->object->velocity,
                        box2->object->velocity,
                        1.0
                    );
                }
            }
        }
    }

    for (size_t i = 0; i < ManagedObjectPool<SphereCollider>::count; i++)
    {
        // Current Collider
        SphereCollider* sphere1 = ManagedObjectPool<SphereCollider>::objects[i];

        // SPHERE-SPHERE COLLISIONS
        for (size_t j = i + 1; j < ManagedObjectPool<SphereCollider>::count; j++)
        {
            // Next Collider
            SphereCollider* sphere2 = ManagedObjectPool<SphereCollider>::objects[j];

            if (sphere1->isStatic && sphere2->isStatic) {
                continue;
            }

            SphereCollisionInfo collisionInfo;
            bool resolveIfNotTrigger = !(sphere1->isTrigger || sphere2->isTrigger);
            if (SpheresColliding(*sphere1, *sphere2, collisionInfo, resolveIfNotTrigger))
            {
                if (Physics::dynamics)
                {
                    if (sphere1->object->isKinematic || sphere2->object->isKinematic) {
                        continue;
                    }
                    if (sphere1->isStatic)
                    {
                        CalculateStaticCollision(collisionInfo.lineOfImpact, sphere2->object->velocity, sphere2->coefficientRestitution);
                    }
                    else if (sphere2->isStatic)
                    {
                        CalculateStaticCollision(collisionInfo.lineOfImpact, sphere1->object->velocity, sphere1->coefficientRestitution);
                    }
                    else
                    {
                        CalculateCollision(
                            collisionInfo.lineOfImpact,
                            sphere1->object->mass,
                            sphere2->object->mass,
                            sphere1->object->velocity,
                            sphere2->object->velocity,
                            1.0
                        );
                    }
                }
            }
        }

        // SPHERE-PLANE COLLISIONS
        for (size_t ii = 0; ii < ManagedObjectPool<PlaneCollider>::count; ii++)
        {
            // Next Collider
            PlaneCollider* plane = ManagedObjectPool<PlaneCollider>::objects[ii];

            if (sphere1->isStatic && plane->isStatic) {
                continue;
            }

            SphereCollisionInfo collisionInfo;
            bool resolveIfNotTrigger = !(sphere1->isTrigger || plane->isTrigger);
            if (SpherePlaneColliding(*sphere1, *plane, collisionInfo, resolveIfNotTrigger))
            {
                if (Physics::dynamics)
                {
                    if (sphere1->object->isKinematic || plane->object->isKinematic) {
                        continue;
                    }
                    if (sphere1->isStatic)
                    {
                        //...
                    }
                    else if (plane->isStatic)
                    {
                        CalculateStaticCollision(collisionInfo.lineOfImpact, sphere1->object->velocity, sphere1->coefficientRestitution);
                    }
                    else
                    {
                        CalculateCollision(
                            collisionInfo.lineOfImpact,
                            sphere1->object->mass,
                            plane->object->mass,
                            sphere1->object->velocity,
                            plane->object->velocity,
                            1.0
                        );
                    }
                }
            }
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
        endPosition = start + direction * distance;
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
   
    // Builds a 3x3 orthogonal matrix with its -Z axis facing the given direction (similar to a camera with no rotation.
    static Matrix3x3 OrthogonalMatrixLookAt(Vec3 direction)
    {
        // Cached staticallly since highly improbable the initial random vector will ever be exactly aligned with direction arg. 
        // Prevent recalculating a random vector every call.
        static Vec3 randomDirection = RandomDirection();

        Vec3 rayZ = direction * -1.0;
        if (rayZ == randomDirection) {
            randomDirection = RandomDirection();
        }
        Vec3 rayX = CrossProduct(rayZ, randomDirection);
        Vec3 rayY = CrossProduct(rayZ, rayX);
        rayX.Normalize();
        rayY.Normalize();
        float rotationMatrix[3][3] = {
            { rayX.x, rayY.x, rayZ.x },
            { rayX.y, rayY.y, rayZ.y },
            { rayX.z, rayY.z, rayZ.z }
        };

        return rotationMatrix;
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
        cam->localPosition += moveDir * accel * deltaTime;
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
        cam->localPosition += velocity * deltaTime;
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
            }
        }
    }

    if (Physics::collisionDetection)
    {
        DetectCollisions();
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

    if (DEBUGGING)
    {
        std::cout << "--------PHYSICS-------" << endl;
        
        string onoff = Physics::dynamics ? "On" : "Off";
        std::cout << "Physics: " << onoff << " (press p)" << endl;

        onoff = Physics::collisionDetection ? "On" : "Off";
        std::cout << "Collisions: " << onoff << " (press \\)" << endl;

        onoff = Physics::gravity ? "On" : "Off";
        std::cout << "Gravity: " << onoff << " (press G)" << endl;

        std::cout << "Sphere Colliders: " << ManagedObjectPool<SphereCollider>::count << endl;
        std::cout << "Box Colliders: " << ManagedObjectPool <BoxCollider>::count << endl;
        std::cout << "Plane Colliders: " << ManagedObjectPool<PlaneCollider>::count << endl;
        std::cout << "Colliders: " << Collider::count << endl;

        std::cout << "--------PLAYER-------" << endl;

        onoff = isKinematic ? "On" : "Off";
        std::cout << "Kinematic: " << onoff << " (press X)" << endl;

        onoff = dampenersActive ? "On" : "Off";
        std::cout << "Inertial Dampeners: " << onoff << " (press Z)" << endl;

        std::cout << "Position: (" << Camera::main->Position().x << ", " << Camera::main->Position().y << ", " << Camera::main->Position().z << ")" << endl;
        std::cout << "Velocity: <" << velocity.x << ", " << velocity.y << ", " << velocity.z << ">" << endl;
    }
}
#endif