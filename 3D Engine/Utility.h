#pragma once
#ifndef UTILITY_H
#define UTILITY_H

#include <math.h>
#include <functional>

#define List std::vector
extern bool DEBUGGING;
const float PI = 3.14159265359f;
const float TAO = 2.0 * PI;
typedef void (*Callback)();

struct Direction
{
    static Vec3 forward;
    static Vec3 back;
    static Vec3 right;
    static Vec3 left;
    static Vec3 up;
    static Vec3 down;
};
Vec3 Direction::forward = Vec3(0, 0, -1);
Vec3 Direction::back = Vec3(0, 0, 1);
Vec3 Direction::right = Vec3(1, 0, 0);
Vec3 Direction::left = Vec3(-1, 0, 0);
Vec3 Direction::up = Vec3(0, 1, 0);
Vec3 Direction::down = Vec3(0, -1, 0);

class PhysicsObject;
class Component// : public Transform
{
public:
    PhysicsObject* object;
};

template <typename T>
void Foreach(List<T*>& objects, std::function<void(T*)>&& callback)
{
    for (size_t i = 0; i < objects.size(); i++)
    {
        auto obj = objects.at(i);
        callback(obj);
    }
}

template <typename T>
void Foreach(List<T>& objects, std::function<void(T*)>&& callback)
{
    for (size_t i = 0; i < objects.size(); i++)
    {
        auto obj = objects.at(i);
        callback(obj);
    }
}

template <typename T>
class ManagedObjectPool
{
public:
    static List<T*> objects;
    static int count;

    ManagedObjectPool(T* obj)
    {
        if (obj)
        {
            ManagedObjectPool::objects.emplace_back(obj);
            count = ManagedObjectPool::objects.size();//count++;
        }
    }
    
    virtual ~ManagedObjectPool()
    {
        for (size_t i = 0; i < ManagedObjectPool<T>::objects.size(); i++)
        {
            if (this == ManagedObjectPool<T>::objects[i]) {
                ManagedObjectPool<T>::objects.erase(ManagedObjectPool<T>::objects.begin() + i);
                count = ManagedObjectPool<T>::objects.size();
                break;
            }
        }
    }

    static void AddToPool(T* obj)
    {
        for (size_t i = 0; i < ManagedObjectPool<T>::objects.size(); i++)
        {
            if (obj == ManagedObjectPool<T>::objects[i]) {
                return;
            }
        }
        
        ManagedObjectPool<T>::objects.emplace_back(obj);
        count = ManagedObjectPool<T>::objects.size();//count++;
    }

    static void RemoveFromPool(T* obj)
    {
        for (size_t i = 0; i < ManagedObjectPool<T>::objects.size(); i++)
        {
            if (obj == ManagedObjectPool<T>::objects[i]) {
                ManagedObjectPool<T>::objects.erase(ManagedObjectPool<T>::objects.begin() + i);
                count = ManagedObjectPool<T>::objects.size();
                return;
            }
        }
    }
    

   
};
template <typename T>
List<T*> ManagedObjectPool<T>::objects = List<T*>();
template <typename T>
int ManagedObjectPool<T>::count = 0;

class Plane
{
private:
    bool pointNormalForm = false;
public:
    Vec3 verts[3];
    Vec3 normal;

    Plane(){}

    Plane(Vec3 p1, Vec3 p2, Vec3 p3)
    {
        verts[0] = p1;
        verts[1] = p2;
        verts[2] = p3;

        Normal();
    }

    //NEEDS TESTING
    Plane(Vec3 pointOnPlane, Vec3 normal)
    {
        pointNormalForm = true;
        this->normal = normal;
        verts[0] = pointOnPlane;
        float D = DotProduct(normal, pointOnPlane);
        
        if (normal.x == 0.0 && normal.y == 0.0 && normal.z != 0.0)
        {
            verts[1] = Vec3(-1, 1, pointOnPlane.z);
            verts[2] = Vec3(1, 1, pointOnPlane.z);
        }
        else if (normal.x == 0.0 && normal.y != 0.0 && normal.z == 0.0)
        {
            verts[1] = Vec3(-1, pointOnPlane.y, 1);
            verts[2] = Vec3(1, pointOnPlane.y, 1);
        }
        else if (normal.x != 0.0 && normal.y == 0.0 && normal.z == 0.0)
        {
            verts[1] = Vec3(pointOnPlane.x, -1, 1);
            verts[2] = Vec3(pointOnPlane.x, 1, 1);
        }
        else
        {
            if (normal.x != 0.0) {
                float x1 = (D - normal.y) / normal.x;// = (D - (Ny*1 + Nz*0)) / Nx
                float x2 = (D - normal.z) / normal.x;// = (D - (Ny*0 + Nz*1)) / Nx
                verts[1] = Vec3(x1, 1, 0);
                verts[2] = Vec3(x2, 0, 1); 
            }
            else if (normal.y != 0.0) {
                float y1 = (D - normal.x) / normal.y;// = (D - (Nx*1 + Nz*0)) / Ny
                float y2 = (D - normal.z) / normal.y;// = (D - (Nx*0 + Nz*1)) / Ny
                verts[1] = Vec3(1, y1, 0);
                verts[2] = Vec3(0, y2, 1);
            }
            else if (normal.z != 0.0) {
                float z1 = (D - normal.x) / normal.z;// = (D - (Nx*1 + Ny*0)) / Nz
                float z2 = (D - normal.y) / normal.z;// = (D - (Nx*0 + Ny*1)) / Nz
                verts[1] = Vec3(1, 0, z1); 
                verts[2] = Vec3(0, 1, z2); 
            }
        }
    }

    Vec3 Normal()
    {
        if (pointNormalForm)
        {
            return normal;
        }

        // Calculate triangle suface Normal
        Vec3 a = verts[2] - verts[0];
        Vec3 b = verts[1] - verts[0];
        normal = (CrossProduct(a, b)).Normalized();

        return normal;
    }
};

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

float Clamp(float value, float min, float max)
{
    if (value < min) {
        value = min;
    }
    else if (value > max) {
        value = max;
    }
    return value;
}

Vec3 RandomVector()
{
    Vec3 randomVector = Vec3(rand(), rand(), rand());

    return randomVector;
}

Vec3 RandomDirection()
{
    Vec3 vec = RandomVector();
    vec.Normalize();

    return vec;
}

float ToDeg(float rad) {
    return rad * 180.0 / PI;
}

float ToRad(float deg) {
    return deg * PI / 180.0;
}

Vec3 Reflect(Vec3 v, Vec3 n)
{
    return v + (n*2 * DotProduct(v*-1, n));
}

Vec3 ProjectOnPlane(Vec3 v, Vec3 n)
{
    Vec3 b = n * DotProduct(n, v);
    return v - b;
}

Range ProjectVertsOntoAxis(Vec3* verts, const int& count, Vec3& axis)
{
    float dist = DotProduct(verts[0], axis);
    float min = dist;
    float max = dist;
    for (size_t k = 1; k < count; k++)
    {
        dist = DotProduct(verts[k], axis);
        if (dist < min) {
            min = dist;
        }
        else if (dist > max) {
            max = dist;
        }
    }

    return Range(min, max);
}

Vec3 ClosestPoint(List<Vec3>& verts, Vec3& pointComparing, float* closestDistance = NULL)
{
    Vec3 closestPoint;
    float closestDist;
    for (size_t i = 0; i < verts.size(); i++)
    {
        if (i == 0) {
            closestPoint = verts[i];
            closestDist = (verts[i]-pointComparing).SqrMagnitude();
            continue;
        }

        Vec3 point = verts[i] - pointComparing;
        float dist = point.SqrMagnitude();
        if (dist < closestDist)
        {
            closestDist = dist;
            closestPoint = verts[i];
        }
    }

    if (closestDistance) {
        *closestDistance = closestDist;
    }

    return closestPoint;
}

// Plane: N_x(x-x0) + N_y(y-y0) + N_z(z-z0) = 0
// Point on plane: x0,y0,z0
// Point on line: x,y,z
// Parametric Line: P = P0 + Vt ---> lineEnd = lineStart + (lineEnd-lineStart)t
// 1st. Paramiterize line like this ---> x = (P0_x + V_x*t), y =, z = ... 
// 2nd. Plugin x,y,z it into plane equation and solve for t.
// 3rd. Use t to find the point intersecting both the line and plane.

bool LinePlaneIntersecting(Vec3& lineStart, Vec3& lineEnd, Vec3& pointOnPlane, Vec3& normal, Vec3* pointIntersecting)
{
    Vec3 v = lineEnd - lineStart;
    float divisor = DotProduct(normal, v);
    if (divisor != 0.0)
    {
        float t = DotProduct(normal, pointOnPlane - lineStart) / divisor;
        //if (t >= 0.0)
        {
            *pointIntersecting = lineStart + (v * t);
            return true;
        }
    }
    return false;
}

bool LinePlaneIntersecting(Vec3& lineStart, Vec3& lineEnd, Plane& plane, Vec3* pointIntersecting)
{
    return LinePlaneIntersecting(lineStart, lineEnd, ((Vec3&)plane.verts[0]), ((Vec3&)plane.Normal()), pointIntersecting);
} 

bool PointInsideTriangle(const Vec3& p, const Vec3 triPoints[3])
{
    Vec3 A = triPoints[0];
    Vec3 B = triPoints[1];
    Vec3 C = triPoints[2];
    float divisor = (((B.x - A.x) * (C.y - A.y)) - ((B.y - A.y) * (C.x - A.x)));
    float divisor2 = (C.y - A.y);
    if (divisor == 0.0 || divisor2 == 0.0)
    {
        return false;
    }
    float w1 = (((p.x - A.x) * (C.y - A.y)) - ((p.y - A.y) * (C.x - A.x))) / divisor;
    float w2 = ((p.y - A.y) - (w1 * (B.y - A.y))) / divisor2;

    return ((w1 >= 0.0 && w2 >= 0.0) && (w1 + w2) <= 1.0);
}

bool PointInsideCube(Vec3& cubeMin, Vec3& cubeMax, Vec3& point)
{
    if (point.x >= cubeMin.x && point.x <= cubeMax.x
        && point.y >= cubeMin.y && point.y <= cubeMax.y
        && point.z >= cubeMin.z && point.z <= cubeMax.z)
    {
        return true;
    }
    return false;
}

//Needs Testing
void PlanesIntersecting(Vec3& normal1, Vec3& p1, Vec3& normal2, Vec3& p2)
{
    float D1 = DotProduct(normal1, p1);
    float D2 = DotProduct(normal2, p2);
    Vec3 v = CrossProduct(normal1, normal2);
    float y = (D2 * normal1.x - D1 * normal2.x) / (-normal2.x * normal1.y + normal2.y);
    float x = (D1 - normal1.y * y) / normal1.x;

    float t = 0;
    Vec3 line = Vec3(x, y, 0) + v * t;
}

Vec3 ClosestPointOnSphere(Vec3& center, float& radius, Vec3& somePoint)
{
    Vec3 dir = (somePoint - center).Normalized();
    return center + dir * radius;
}

Vec3 ClosestPointOnPlane(Vec3& pointOnPlane, Vec3& normal, Vec3& somePoint)
{
    Vec3 v = somePoint - pointOnPlane;
    Vec3 vPerp = normal * (DotProduct(v, normal));
    return somePoint - vPerp;
}

// Builds a 3x3 orthogonal matrix with its -Z axis facing the given direction (similar to a camera with no rotation).
Matrix3x3 OrthogonalMatrixLookAt(Vec3 direction)
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
    //rayX.Normalize();
    //rayY.Normalize();
    float rotationMatrix[3][3] = {
        { rayX.x, rayY.x, rayZ.x },
        { rayX.y, rayY.y, rayZ.y },
        { rayX.z, rayY.z, rayZ.z }
    };

    return rotationMatrix;
}

Matrix3x3 SkewSymmetric3x3(const Vec3& w)
{
    float matrix[][3] = {
        {0, -w.z, w.y},
        {w.z, 0, -w.x},
        {-w.y, w.x, 0}
    };

    return matrix;
}

Matrix3x3 SkewSymmetric3x3(Vec3& dir, float& radians)
{
    return SkewSymmetric3x3(dir * radians);
}

Matrix3x3 MatrixDot(Vec3& dir)
{
    float xSqr = dir.x*dir.x;
    float ySqr = dir.y *dir.y;
    float zSqr = dir.z*dir.z;
    float xy = dir.x * dir.y;
    float xz = dir.x * dir.z;
    float yz = dir.y * dir.z;

    float matrix[][3] = {
        {xSqr, xy, xz},
        {xy, ySqr, yz},
        {xz, yz, zSqr}
    };

    return matrix;
}
#endif