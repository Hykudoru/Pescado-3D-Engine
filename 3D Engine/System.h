#pragma once

#include <math.h>
#include <functional>

using namespace std;
#define List std::vector
class PhysicsObject;
class Collider;
class Mesh;
class Component// : public Transform
{
public:
    PhysicsObject* object;
};

extern bool DEBUGGING;
const float PI = 3.14159265359f;
const float TAO = 2.0 * PI;
typedef void (*Callback)();

List<std::string> debugLog = List<std::string>();
List<std::string> debugLog2 = List<std::string>();

void Log(const std::string& str)
{
    debugLog.emplace_back(str);
}

void Log2(const std::string& str)
{
    debugLog2.emplace_back(str);
}

void ClearLog()
{
    debugLog.clear();
}

void ClearLog2()
{
    debugLog2.clear();
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

    static void RemoveAll()
    {
        ManagedObjectPool<T>::objects.erase(ManagedObjectPool<T>::objects.begin(), ManagedObjectPool<T>::objects.end());
        count = 0;
    }
};
template <typename T>
List<T*> ManagedObjectPool<T>::objects = List<T*>();
template <typename T>
int ManagedObjectPool<T>::count = 0;


// To-do
void UnloadCurrentScene()
{
    ManagedObjectPool<PhysicsObject>::RemoveAll();
    ManagedObjectPool<Collider>::RemoveAll();
    ManagedObjectPool<Mesh>::RemoveAll();
}

// To-do
void LoadScene(string sceneFilePath)
{
    UnloadCurrentScene();
    // Parse scene data and allocate
}

// To-do
void SaveCurrentScene()
{

}