#pragma once
#include<string>
#ifndef VECTOR_H
#define VECTOR_H

template <typename T>
class Vector3;

template <typename T>
class Vector4;

template <typename T>
T DotProduct(Vector3<T> a, Vector3<T> b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

template <typename T>
T DotProduct(Vector4<T> a, Vector4<T> b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
}

// Remember order matters. A X B != B X A
// Calculates a unit vector orthogonal/perpendicular to both A and B vectors
template <typename T>
Vector3<T> CrossProduct(Vector3<T> a, Vector3<T> b)
{
    /* Matrix
     |  i | j | k  |
     | a.x a.y a.z |
     | b.x b.y b.z |
    */

    Vector3<T> u;
    u.x = (a.y * b.z - a.z * b.y); //i 
    u.y = -(a.x * b.z) + (a.z * b.x);// -(a.x*b.z - a.z*b.x), //-j
    u.z = (a.x * b.y - a.y * b.x); //k
    /* To check is orthogonal: bool isPerpendicular = dotProduct(u, a) == 0 && dotProduct(u, b) == 0;
    */
    return u;
}

//-------------------------------
// --------- Vector2 ------------
//-------------------------------
template <typename T>
class Vector2
{
public:
    T x;
    T y;

    Vector2();
    Vector2(T xVal, T yVal);
    Vector2(T xy[]);

    T SqrMagnitude()
    {
        return x * x + y * y;
    }

    T Magnitude()
    {
        return sqrt(this->SqrMagnitude());
    }

    //--------- Normalize() vs Normalized() ---------
    // Normalize() modifies the original
    // Normalized() returns a normalized vector without modifying the original.

    void Normalize()
    {
        float length = this->Magnitude();
        if (length < 0.00001) {
            x = 0;
            y = 0;
        }
        else {
            x /= length;
            y /= length;
        }
    }

    Vector2<T> Normalized()
    {
        float length = this->Magnitude();
        return Vector2<T>((float)x / length, (float)y / length);
    }

    Vector2 operator+(const Vector2& other)
    {
        Vector2 vectorSum;
        vectorSum.x = this->x + other.x;
        vectorSum.y = this->y + other.y;
        return vectorSum;
    }

    Vector2 operator-(const Vector2& other)
    {
        Vector2 vectorDiff;
        vectorDiff.x = this->x - other.x;
        vectorDiff.y = this->y - other.y;
        return vectorDiff;
    }

    Vector2 operator*(const T& scalar)
    {
        Vector2 scaledVector;
        scaledVector.x = this->x * scalar;
        scaledVector.y = this->y * scalar;
        return scaledVector;
    }

    Vector2& operator+=(const Vector2& other)
    {
        this->x += other.x;
        this->y += other.y;
        return *this;
    }

    Vector2& operator-=(const Vector2& other)
    {
        this->x -= other.x;
        this->y -= other.y;
        return *this;
    }

    Vector2& operator*=(const T scalar)
    {
        this->x *= scalar;
        this->y *= scalar;
        return *this;
    }

    Vector2& operator/=(const T divisor)
    {
        if (divisor != 0.0) {
            this->x /= divisor;
            this->y /= divisor;
        }
        return *this;
    }

    operator Vector3<T>();
};

// Constructors
template <typename T>
Vector2<T>::Vector2()
{
    x = 0.0;
    y = 0.0;
}
template <typename T>
Vector2<T>::Vector2(T xVal, T yVal)
{
    x = xVal;
    y = yVal;
}

template <typename T>
Vector2<T>::Vector2(T xy[])
{
    x = xy[0];
    y = xy[1];
}

// ----------------------- Vector3 -------------------------
template <typename T>
class Vector3
{
public:
   // T v[3];
    T x;
    T y;
    T z;
    int size = 3;
    //static const Zero = new Vec3(0, 0, 0);
    //static const One = new Vec3(1.0, 1.0, 1.0);

    Vector3();
    Vector3(T xVal, T yVal, T zVal);
    Vector3(T xyz[]);
    /*
    string ToString() 
    {
        string str = "";
        str += "(" + x + ", " + y + ", " + z + ")";
        str += "\n\r";
        return str;
    }*/

    T SqrMagnitude()
    {
        return x * x + y * y + z * z;
    }

    T Magnitude()
    {
        return sqrt(this->SqrMagnitude());
    }

    //--------- Normalize() vs Normalized() ---------
    // Normalize() modifies the original
    // Normalized() returns a normalized vector without modifying the original.

    void Normalize()
    {
        float length = this->Magnitude();
        if (length != 0.0) 
        {
            x /= length;
            y /= length;
            z /= length;
        }
    }

    Vector3<T> Normalized()
    {
        float length = this->Magnitude();
        Vector3<T> norm;
        if (length != 0.0)
        {
            norm.x = x/length;
            norm.y = y/length;
            norm.z = z/length;
        }
        
        return norm;
    }

    Vector3 operator+(const Vector3& other)
    {
        Vector3 vectorSum;
        vectorSum.x = this->x + other.x;
        vectorSum.y = this->y + other.y;
        vectorSum.z = this->z + other.z;
        return vectorSum;
    }

    Vector3 operator-(const Vector3& other)
    {
        Vector3 vectorDiff;
        vectorDiff.x = this->x - other.x;
        vectorDiff.y = this->y - other.y;
        vectorDiff.z = this->z - other.z;
        return vectorDiff;
    }

    Vector3 operator*(const T& scalar)
    {
        Vector3 scaledVector;
        scaledVector.x = this->x * scalar;
        scaledVector.y = this->y * scalar;
        scaledVector.z = this->z * scalar;
        return scaledVector;
    }

    Vector3& operator+=(const Vector3& other)
    {
        this->x += other.x;
        this->y += other.y;
        this->z += other.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& other)
    {
        this->x -= other.x;
        this->y -= other.y;
        this->z -= other.z;
        return *this;
    }

    Vector3& operator*=(const T scalar)
    {
        this->x *= scalar;
        this->y *= scalar;
        this->z *= scalar;
        return *this;
    }

    Vector3& operator/=(const T divisor)
    {
        if (divisor != 0.0)
        {
            this->x /= divisor;
            this->y /= divisor;
            this->z /= divisor;
            return *this;
        }
    }

    operator Vector2<T>();
    operator Vector4<T>();
};

// Casting
template <typename T>
Vector2<T>::operator Vector3<T>()
{
    Vector3<T> vec3;
    vec3.x = x;
    vec3.y = y;
    vec3.z = 0.0;
    return vec3;
}
// Casting
template <typename T>
Vector3<T>::operator Vector2<T>()
{
    Vector2<T> vec2;
    vec2.x = x;
    vec2.y = y;
    return vec2;
}

// Constructors
template <typename T>
Vector3<T>::Vector3()
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
}
template <typename T>
Vector3<T>::Vector3(T xVal, T yVal, T zVal)
{
    x = xVal;
    y = yVal;
    z = zVal;
}

template <typename T>
Vector3<T>::Vector3(T xyz[])
{
    x = xyz[0];
    y = xyz[1];
    z = xyz[2];
}

// ----------------------- Vector4 -------------------------
template <typename T>
class Vector4
{
public:
    T x = 0.0;
    T y = 0.0;
    T z = 0.0;
    T w = 1.0;

    Vector4();
    Vector4(T xVal, T yVal, T zVal, T wVal = 1.0);
    Vector4(T xyzw[]);

    operator Vector3<T>();
};

//================================
// Casting
template <typename T>
Vector3<T>::operator Vector4<T>()
{
    Vector4<T> vec4;
    vec4.x = x;
    vec4.y = y;
    vec4.z = z;
    vec4.w = 1.0;
    return vec4;
}
// IMPORTANT - Casting 4D Homogeneous to 3D Cartesian coordinates results in perspective divide --------
template <typename T>
Vector4<T>::operator Vector3<T>()
{
    /*Vector3<T> vec3;
    if (w != 0.0) {
        vec3.x = x/w;
        vec3.y = y/w;
        vec3.z = z/w;
    }*/
    Vector3<T> vec3(x, y, z);

    return vec3;
}
// Constructors
template <typename T>
Vector4<T>::Vector4()
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    w = 1.0;
}
template <typename T>
Vector4<T>::Vector4(T xVal, T yVal, T zVal, T wVal)
{
    x = xVal;
    y = yVal;
    z = zVal;
    w = wVal;
}

template <typename T>
Vector4<T>::Vector4(T xyzw[])
{
    x = xyzw[0];
    y = xyzw[1];
    z = xyzw[2];
    w = xyzw[3];
}

#define Vec2 Vector3<float>
#define Vec3 Vector3<float>
#define Vec4 Vector4<float>

#endif

/*
using Vector3f = Vector3<float>;
float vec3[];
Vector3f m[3][3] = {
    {Vector3f()},
    {Vector3f()},
    {Vector3f()}
};
float identity[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1},
};

space coordinates xyz
body coordinates XYZ-++
*/