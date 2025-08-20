#pragma once
#include <Matrix.h>
#include <vector>
#include <algorithm>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <Utility.h>;
#ifndef GRAPHICS_H
#define GRAPHICS_H

class Plane;
struct Point;
struct Line;
struct Triangle;
class Transform;
class Mesh;
class Camera;

Vec3 lightSource = .25 * Direction::up + Direction::back * .5;
static float worldScale = 1;
int screenWidth = 1920;//1600;
int screenHeight = 1080;// 900;
float nearClippingPlane = -0.1;
float farClippingPlane = -100000.0;
float fieldOfViewDeg = 60;
float fov = ToRad(fieldOfViewDeg);
float aspect = (float)screenHeight / (float)screenWidth;

List<Point>* pointBuffer = new List<Point>();
List<Line>* lineBuffer = new List<Line>();
List<Triangle>* triBuffer = new List<Triangle>();

struct Color
{
    float r;
    float g;
    float b;
    float a;

    static Color black;
    static Color white;
    static Color gray;
    static Color red;
    static Color green;
    static Color blue;
    static Color pink;
    static Color purple;
    static Color yellow;
    static Color turquoise;
    static Color orange;

    Color()
    {
        this->r = 0.0;
        this->g = 0.0;
        this->b = 0.0;
        this->a = 1;
    }

    Color(float r, float g, float b, float a = 1.0)
    {
        this->r = r;
        this->g = g;
        this->b = b;
        this->a = a;
    }

    Color(Vec4 vec)
    {
        this->r = vec.x;
        this->g = vec.y;
        this->b = vec.z;
        this->a = vec.w;
    }

    static Color Random()
    {
        Color c = Color(Clamp(rand(), 0, 255), Clamp(rand(), 0, 255), Clamp(rand(), 0, 255));
        return c;
    }

    Color operator+(const Color& other)
    {
        Color color;
        color.r = Clamp(this->r + other.r, 0, 255);
        color.g = Clamp(this->g + other.g, 0, 255);
        color.b = Clamp(this->b + other.b, 0, 255);
        return color;
    }

    Color operator-(const Color& other)
    {
        Color color;
        color.r = Clamp(this->r - other.r, 0, 255);
        color.g = Clamp(this->g - other.g, 0, 255);
        color.b = Clamp(this->b - other.b, 0, 255);
        return color;
    }

    Color operator+(const Vec3& v3)
    {
        Color color;
        color.r = Clamp(this->r + v3.x, 0, 255);
        color.g = Clamp(this->g + v3.y, 0, 255);
        color.b = Clamp(this->b + v3.z, 0, 255);
        return color;
    }

    Color operator-(const Vec3& v3)
    {
        Color color;
        color.r = Clamp(this->r - v3.x, 0, 255);
        color.g = Clamp(this->g - v3.y, 0, 255);
        color.b = Clamp(this->b - v3.z, 0, 255);
        return color;
    }

    Color operator*(const float& scalar)
    {
        Color color;
        color.r = Clamp(this->r * scalar, 0, 255);
        color.g = Clamp(this->g * scalar, 0, 255);
        color.b = Clamp(this->b * scalar, 0, 255);
        return color;
    }

    Color& operator+=(const Color& other)
    {
        this->r = Clamp(this->r + other.r, 0, 255);
        this->g = Clamp(this->g + other.g, 0, 255);
        this->b = Clamp(this->b + other.b, 0, 255);
        return *this;
    }

    Color& operator+=(const Vec3& v3)
    {
        this->r = Clamp(this->r + v3.x, 0, 255);
        this->g = Clamp(this->g + v3.y, 0, 255);
        this->b = Clamp(this->b + v3.z, 0, 255);
        return *this;
    }

    Color& operator-=(const Color& other)
    {
        this->r = Clamp(this->r - other.r, 0, 255);
        this->g = Clamp(this->g - other.g, 0, 255);
        this->b = Clamp(this->b - other.b, 0, 255);
        return *this;
    }

    Color& operator-=(const Vec3& v3)
    {
        this->r = Clamp(this->r - v3.x, 0, 255);
        this->g = Clamp(this->g - v3.y, 0, 255);
        this->b = Clamp(this->b - v3.z, 0, 255);
        return *this;
    }

    Color& operator*=(const float scalar)
    {
        this->r = Clamp(this->r * scalar, 0, 255);
        this->g = Clamp(this->g * scalar, 0, 255);
        this->b = Clamp(this->b * scalar, 0, 255);
        return *this;
    }

    Color& operator/=(const float divisor)
    {
        if (divisor != 0.0)
        {
            this->r = Clamp(this->r / divisor, 0, 255);
            this->g = Clamp(this->g / divisor, 0, 255);
            this->b = Clamp(this->b / divisor, 0, 255);
            return *this;
        }
    }
    operator Vec3();
};

Color::operator Vec3()
{
    Vec3 vec(r, g, b);
    return vec;
}

Color Color::black = Color(0, 0, 0);
Color Color::white = Color(255, 255, 255);
Color Color::gray = Color(128, 128, 128);
Color Color::red = Color(255, 0, 0);
Color Color::green = Color(0, 255, 0);
Color Color::blue = Color(0, 0, 255);
Color Color::pink = Color(255, 0, 255);
Color Color::purple = Color(128, 0, 128);
Color Color::yellow = Color(255, 255, 0);
Color Color::turquoise = Color(0, 255, 255);
Color Color::orange = Color(255, 158, 0);

struct Material
{
    std::string name = "";
    Color color = Color::white;

    Material(std::string id = "", Color color = Color::white)
    {
        this->name = id;
        this->color = color;
    }
};

class Transform
{
protected:
    Transform* parent = nullptr;
    Transform* root = nullptr;
public:
    Vec3 localScale = Vec3(1, 1, 1);
    Vec3 localPosition = Vec3(0, 0, 0);
    Matrix3x3 localRotation = Matrix3x3::identity;

    Transform(const float& scale = 1, const Vec3& position = Vec3(0, 0, 0), const Vec3& rotationEuler = Vec3(0, 0, 0))
    {
        this->localScale.x = scale;
        this->localScale.y = scale;
        this->localScale.z = scale;
        this->localPosition = position;
        this->localRotation = YPR(rotationEuler.x, rotationEuler.y, rotationEuler.z);
        this->root = this;
    }

    Transform(const Vec3& scale, const Vec3& position = Vec3(0, 0, 0), const Matrix3x3& rotation = Matrix3x3::identity)
    {
        this->localScale = scale;
        this->localPosition = position;
        this->localRotation = rotation;
        this->root = this;
    }

    Vec3 Forward() { return Rotation() * Direction::forward; }
    Vec3 Back() { return Rotation() * Direction::back; }
    Vec3 Right() { return Rotation() * Direction::right; }
    Vec3 Left() { return Rotation() * Direction::left; }
    Vec3 Up() { return Rotation() * Direction::up; }
    Vec3 Down() { return Rotation() * Direction::down; }

    Matrix3x3 Rotation();

    Vec3 Position();

    Vec3 Scale();

    void SetParent(Transform* newParent, bool changeOfBasisTransition = true);

    Transform& Parent() { return *this->parent; }

    Transform& Root() { return *this->root; }

    Matrix4x4 LocalScale4x4();

    Matrix4x4 LocalScale4x4Inverse();

    Matrix4x4 LocalRotation4x4();

    Matrix4x4 LocalTranslation4x4();

    Matrix4x4 LocalTranslation4x4Inverse();

    // 1:Scale, 2:Rotate, 3:Translate
    Matrix4x4 TRS();

    // S^-1 * R^-1 * T^-1
    Matrix4x4 TRSInverse();

    // 1:Rotate, 2:Translate
    Matrix4x4 TR();

    // R^-1 * T^-1
    Matrix4x4 TRInverse();
};

class Cube : public Transform
{
public:
    List<Vec3> vertices;
    Vec3 min;
    Vec3 max;
    Cube()
    {
        vertices = List<Vec3>({//new Vec3[8] {
            //south
            Vec3(-0.5, -0.5, 0.5),
            Vec3(-0.5, 0.5, 0.5),
            Vec3(0.5, 0.5, 0.5),
            Vec3(0.5, -0.5, 0.5),
            //north
            Vec3(-0.5, -0.5, -0.5),
            Vec3(-0.5, 0.5, -0.5),
            Vec3(0.5, 0.5, -0.5),
            Vec3(0.5, -0.5, -0.5)
        });
        min = Vec3(-0.5, -0.5, -0.5);
        max = Vec3(0.5, 0.5, 0.5);
    }

    Cube(const Vec3& min, const Vec3& max)
    {
        vertices = List<Vec3>(8);
        //south
        vertices[0] = { min.x, min.y, max.z };  //Vec3(-0.5, -0.5, 0.5),
        vertices[1] = { min.x, max.y, max.z };  //Vec3(-0.5, 0.5, 0.5),
        vertices[2] = { max.x, max.y, max.z };  //Vec3(0.5, 0.5, 0.5),
        vertices[3] = { max.x, min.y, max.z };  // Vec3(0.5, -0.5, 0.5),
        //north
        vertices[4] = { min.x, min.y, min.z };  //Vec3(-0.5, -0.5, -0.5),
        vertices[5] = { min.x, max.y, min.z };  //Vec3(-0.5, 0.5, -0.5),
        vertices[6] = { max.x, max.y, min.z };  //Vec3(0.5, 0.5, -0.5),
        vertices[7] = { max.x, min.y, min.z };  // Vec3(0.5, -0.5, -0.5),

        this->min = min;
        this->max = max;
    }

    Cube(const float& min, const float& max)
    {
        vertices = List<Vec3>(8);

        //south
        vertices[0] = { min, min, max };  //Vec3(-0.5, -0.5, 0.5),
        vertices[1] = { min, max, max };  //Vec3(-0.5, 0.5, 0.5),
        vertices[2] = { max, max, max };  //Vec3(0.5, 0.5, 0.5),
        vertices[3] = { max, min, max };  // Vec3(0.5, -0.5, 0.5),
        //north
        vertices[4] = { min, min, min };  //Vec3(-0.5, -0.5, -0.5),
        vertices[5] = { min, max, min };  //Vec3(-0.5, 0.5, -0.5),
        vertices[6] = { max, max, min };  //Vec3(0.5, 0.5, -0.5),
        vertices[7] = { max, min, min };  // Vec3(0.5, -0.5, -0.5),

        this->min = Vec3(min, min, min);
        this->max = Vec3(max, max, max);
    }
};

class BoundingBox : public Transform, public ManagedObjectPool<BoundingBox>
{
public:
    Cube bounds;
    Vec3 min;
    Vec3 max;
    Color color = Color::red;
    Mesh* mesh;

    BoundingBox(Mesh* mesh) : ManagedObjectPool<BoundingBox>(this)
    {
        this->mesh = mesh;
        CreateBounds(mesh);
    }

    void CreateBounds(Mesh* mesh);

    List<Vec3>* WorldVertices();
    List<Vec3>* ViewspaceVertices();
    List<Vec3>* ProjectedVertices();

    void Draw();
};

class Mesh : public Component, public Transform, public ManagedObjectPool<Mesh>
{
protected:
    Color color = Color::white;
public:
    static int worldTriangleDrawCount;
    List<Vec3> vertices;
    List<int>* indices;
    List<Triangle>* triangles;
    bool ignoreLighting = false;
    bool forceWireFrame = false;
    //Mesh(const Mesh& other) = delete;//disables copying
    BoundingBox* bounds;

    Mesh(const float& scale = 1, const Vec3& position = Vec3(0, 0, 0), const Vec3& rotationEuler = Vec3(0, 0, 0))
        : Transform(scale, position, rotationEuler), ManagedObjectPool<Mesh>(this)
    {
        triangles = new List<Triangle>{};
        MapVertsToTriangles();
        SetColor(color);
        bounds = new BoundingBox(this);
    }
    
    Mesh(const Vec3& scale, const Vec3& position = Vec3(0, 0, 0), const Matrix3x3& rotation = Matrix3x3::identity)
        : Transform(scale, position, rotation), ManagedObjectPool<Mesh>(this)
    {
        triangles = new List<Triangle>{};
        MapVertsToTriangles();
        SetColor(color);
        bounds = new BoundingBox(this);
    }

    virtual ~Mesh()
    {
        //delete vertices;
        delete indices;
        delete triangles;
        delete bounds;
    }

    bool SetVisibility(bool visible);

    void SetColor(Color&& c);
    void SetColor(Color& c);
    Color GetColor() { return this->color; }

    virtual List<Triangle>* MapVertsToTriangles();

    //Convert to world coordinates
    List<Vec3> WorldVertices();

    void TransformTriangles();
};

struct Graphics
{
    static bool frustumCulling;
    static bool backFaceCulling;
    static bool invertNormals;
    static bool debugNormals;
    static bool debugVertices;
    static bool debugAxes;
    static bool debugBounds;
    static bool debugBoxCollisions;
    static bool debugSphereCollisions;
    static bool debugPlaneCollisions;
    static bool debugRaycasting;
    static bool debugTree;
    static bool perspective;
    static bool fillTriangles;
    static bool displayWireFrames;
    static bool lighting;
    static bool vfx;
    static bool matrixMode;

    static void SetDrawColor(Color color)
    {
        glColor4ub(color.r, color.g, color.b, color.a);
    }

    static void SetDrawColor(float r, float g, float b, float a = 1)
    {
        glColor4ub(r, g, b, a);
    }

    static void SetLineWidth(int width)
    {
        glLineWidth(width);
    }

    static void SetPointSize(int size)
    {
        glPointSize(size);
    }

    static void DrawPoint(Vec2 point)
    {
        glBegin(GL_POINTS);
        glVertex2f(point.x, point.y);
        glEnd();
    }

    static void DrawLine(Vec2 from, Vec2 to)
    {
        glBegin(GL_LINES);
        glVertex2f(from.x, from.y);
        glVertex2f(to.x, to.y);
        glEnd();
    }

    static void DrawTriangle(Vec2 p1, Vec2 p2, Vec2 p3)
    {
        glBegin(GL_LINES);
        glVertex2f(p1.x, p1.y);
        glVertex2f(p2.x, p2.y);

        glVertex2f(p2.x, p2.y);
        glVertex2f(p3.x, p3.y);

        glVertex2f(p3.x, p3.y);
        glVertex2f(p1.x, p1.y);
        glEnd();
    }

    static void DrawTriangleFilled(Vec2 p1, Vec2 p2, Vec2 p3)
    {
        glBegin(GL_TRIANGLES);
        glVertex2f(p1.x, p1.y);
        glVertex2f(p2.x, p2.y);
        glVertex2f(p3.x, p3.y);
        glEnd();
    }
};
bool Graphics::frustumCulling = true;
bool Graphics::backFaceCulling = true;
bool Graphics::invertNormals = false;
bool Graphics::debugNormals = false;
bool Graphics::debugVertices = false;
bool Graphics::debugAxes = false;
bool Graphics::debugBounds = false;
bool Graphics::debugBoxCollisions = false;
bool Graphics::debugSphereCollisions = false;
bool Graphics::debugPlaneCollisions = false;
bool Graphics::debugRaycasting = false;
bool Graphics::debugTree = false;
bool Graphics::perspective = true;
bool Graphics::fillTriangles = true;
bool Graphics::displayWireFrames = false;
bool Graphics::lighting = true;
bool Graphics::vfx = false;
bool Graphics::matrixMode = false;

// Perspective Projection Matrix
float persp[4][4] = {
    {aspect * 1 / tan(fov / 2), 0, 0, 0},
    {0, 1 / tan(fov / 2), 0, 0},
    {0, 0, 1, 0},
    {0, 0, -1, 0}
};

// Perspective Projection Matrix
float weakPersp[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, -1, 0}
};

//Orthographic Projection Matrix
float ortho[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 1}
};

float undoAspect[4][4] = {
    {1 / aspect, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 0}
};

Matrix4x4 perspectiveProjectionMatrix = persp;
Matrix4x4 weakPerspectiveProjectionMatrix = weakPersp;
Matrix4x4 orthographicProjectionMatrix = ortho;
Matrix4x4 undoAspectRatio = undoAspect;

void FOV(int deg)
{
    fieldOfViewDeg = deg;
    fov = ToRad(deg);
    float newPerspectiveProjectionMatrix[4][4] = {
        {aspect * 1 / tan(fov / 2), 0, 0, 0},
        {0, 1 / tan(fov / 2), 0, 0},
        {0, 0, 1, 0},
        {0, 0, -1, 0}
    };

    perspectiveProjectionMatrix = newPerspectiveProjectionMatrix;
}

Matrix4x4 ProjectionMatrix()
{
    if (Graphics::perspective) {
        return perspectiveProjectionMatrix;
    }
    else {
        return orthographicProjectionMatrix;
    }
}

Matrix4x4 worldToViewMatrix;
Matrix4x4 projectionMatrix;

struct Point
{
    Vec3 position;
    Color color;
    int size;

    Point(Vec3 position, Color color = Color::white, int size = 2)
    {
        this->position = position;
        this->color = color;
        this->size = size;
    }

    void Draw()
    {
        Graphics::SetPointSize(size);
        Graphics::SetDrawColor(color);
        Graphics::DrawPoint(position);
    }

    static void AddPoint(Point point)
    {
        pointBuffer->emplace_back(point);
    }

    static void AddWorldPoint(Point point)
    {
        auto matrix = ProjectionMatrix() * worldToViewMatrix;
        point.position = matrix * point.position;
        pointBuffer->emplace_back(point);
    }
};

struct Line
{
    Vec3 from;
    Vec3 to;
    Color color;
    int width;

    Line(Vec3 from, Vec3 to, Color color = Color::white, int width = 2)
    {
        this->from = from;
        this->to = to;
        this->color = color;
        this->width = width;
    }

    void Draw()
    {
        Graphics::SetLineWidth(width);
        Graphics::SetDrawColor(color.r, color.g, color.b);
        Graphics::DrawLine(from, to);
    }

    static void AddLine(Line line)
    {
        lineBuffer->emplace_back(line);
    }

    static void AddWorldLine(Line line)
    {
        auto matrix = ProjectionMatrix() * worldToViewMatrix;
        line.from = matrix * line.from;
        line.to = matrix * line.to;
        lineBuffer->emplace_back(line);
    }
};

struct Triangle : Plane
{
    Vec4 centroid = Vec4();
    Color color = Color::white;
    bool forceWireFrame = false;
    Mesh* mesh = nullptr;
    
    Triangle() : Plane()
    {
        centroid = Vec4();
        color = Color::white;
        mesh = nullptr;
    }
    Triangle(Vec3 p1, Vec3 p2, Vec3 p3, Mesh* owner = nullptr) : Plane(p1, p2, p3)
    {
        color = Color::white;
        centroid = Centroid();
        mesh = owner;
    }

    Vec4 Centroid()
    {
        centroid = Vec4(
            (verts[0].x + verts[1].x + verts[2].x) / 3.0,
            (verts[0].y + verts[1].y + verts[2].y) / 3.0,
            (verts[0].z + verts[1].z + verts[2].z) / 3.0,
            centroid.w
        );

        return centroid;
    }

    void Draw()
    {
        Vec4 p1 = verts[0];
        Vec4 p2 = verts[1];
        Vec4 p3 = verts[2];

        if (Graphics::fillTriangles == false)
        {
            Graphics::displayWireFrames = true;
        }

        //glColor3ub(255, 255, 255);
        if (Graphics::matrixMode)
        {
            Graphics::SetDrawColor(0, 255, 0);
        }

        if (Graphics::fillTriangles)
        {
            Graphics::SetDrawColor(color.r, color.g, color.b);
            Graphics::DrawTriangleFilled(p1, p2, p3);
        }

        bool drawWireFrame = Graphics::displayWireFrames || forceWireFrame || (mesh != nullptr ? mesh->forceWireFrame : false);
        if (drawWireFrame)
        {
            if (Graphics::fillTriangles)
            {
                float c = Clamp(1.0 / (0.000001 + (color.r + color.g + color.b) / 3), 0, 255);
                Graphics::SetDrawColor(c, c, c);
            }
            Graphics::DrawLine(p1, p2);
            Graphics::DrawLine(p2, p3);
            Graphics::DrawLine(p3, p1);
        }

        Graphics::SetDrawColor(255, 255, 255);
    }
};

//-------------------------------TRANSFORM---------------------------------------------

Vec3 ExtractPosition(const Matrix4x4& trs)
{
    return { trs.m[0][3], trs.m[1][3], trs.m[2][3] };
}

Vec3 ExtractScale(const Matrix4x4& trs) {
    Vec3 c1 = { trs.m[0][0], trs.m[1][0], trs.m[2][0] };
    Vec3 c2 = { trs.m[0][1], trs.m[1][1], trs.m[2][1] };
    Vec3 c3 = { trs.m[0][2], trs.m[1][2], trs.m[2][2] };
    return { c1.Magnitude(), c2.Magnitude(), c3.Magnitude() };
}

// Pass scale parameter if already known since finding the rotation matrix for a TRS matrix 
// requires the scale vector, which finding can be expensive.
Matrix3x3 ExtractRotation(const Matrix4x4 trs, Vec3* scale = NULL)
{
    static Vec3 v = Vec3::zero;
    Vec3 *s = &v;
    if (scale == NULL) {
        *s = ExtractScale(trs);
    }
    else {
        s = scale;
    }

    float rot[3][3] = {
        {trs.m[0][0] / s->x, trs.m[1][0] / s->y, trs.m[2][0] / s->z},
        {trs.m[0][1] / s->x, trs.m[1][1] / s->y, trs.m[2][1] / s->z},
        {trs.m[0][2] / s->x, trs.m[1][2] / s->y, trs.m[2][2] / s->z},
    };

    return rot;
}

struct TRSInfo
{
    Vec3 scale;
    Vec3 position;
    Matrix3x3 rotation;

    TRSInfo(const Vec3& s, const Vec3& pos, const Matrix3x3& rot)
    {
        scale = s;
        position = pos;
        rotation = rot;
    }
};

TRSInfo ExtractTRS(Matrix4x4& trs)
{
    Vec3 scale = ExtractScale(trs);
    return TRSInfo(scale, ExtractPosition(trs), ExtractRotation(trs, &scale));
}

Matrix3x3 Transform::Rotation()
{
    if (parent) {//TRS already checks for parent but checks again here because cheaper to return local position than creating the matrix
        return ExtractRotation(TRS());
    }
    return localRotation;
}

Vec3 Transform::Position()
{
    if (parent) {//TRS already checks for parent but checks again here because cheaper to return local position than creating the matrix
        return ExtractPosition(TRS());
    }

    return localPosition;
}

Vec3 Transform::Scale()
{
    if (parent) {
        return ExtractScale(TRS());
    }
    return localScale;
}

void Transform::SetParent(Transform* newParent, bool changeOfBasisTransition)
{
    static Matrix4x4 T;
    if (newParent == nullptr)
    {
        if (changeOfBasisTransition)
        {
            // Results in seemless unparenting. Assigns (possibly parented) global values to the local values. 
            // This way of unparenting will maintain the previous parented position and rotation but in the new reference frame.
            // Also nothing changes if never parented.
            //if already had a parent
            
            T = this->TR();
            
           // this->scale = Scale();// Vec3(1, 1, 1);
            float rot[3][3] = {
                {T.m[0][0], T.m[0][1], T.m[0][2]},
                {T.m[1][0], T.m[1][1], T.m[1][2]},
                {T.m[2][0], T.m[2][1], T.m[2][2]}
            };
            this->localRotation = rot;
            this->localPosition = Vec3(T.m[0][3], T.m[1][3], T.m[2][3]);
        }

        this->parent = NULL;
        this->root = this;
    }
    else if (newParent != this)
    {
        if (changeOfBasisTransition)
        {
            // If you immediatley parent a transform, everything is calculated relative to its immediate parent's reference frame. 
            // This would result in a transformation if the original coordinates didn't change. 
            // Instead what we want is a change of basis.
            T = newParent->TRInverse() * this->TR();
            
            float rot[3][3] = {
                {T.m[0][0], T.m[0][1], T.m[0][2]},
                {T.m[1][0], T.m[1][1], T.m[1][2]},
                {T.m[2][0], T.m[2][1], T.m[2][2]}
            };
            this->localScale = this->Scale();
            this->localRotation = rot;
            this->localPosition = Vec3(T.m[0][3], T.m[1][3], T.m[2][3]);
            
        }

        this->parent = newParent;
        this->root = newParent->root;
    }
}

Matrix4x4 Transform::LocalScale4x4()
{
    float matrix[4][4] =
    {
        {this->localScale.x, 0, 0, 0},
        {0, this->localScale.y, 0, 0},
        {0, 0, this->localScale.z, 0},
        {0, 0, 0, 1}
    };

    return Matrix4x4(matrix);
}

Matrix4x4 Transform::LocalScale4x4Inverse()
{
    float inverse[4][4] =
    {
        {1.0 / this->localScale.x, 0, 0, 0},
        {0, 1.0 / this->localScale.y, 0, 0},
        {0, 0, 1.0 / this->localScale.z, 0},
        {0, 0, 0, 1}
    };

    return Matrix4x4(inverse);
}

Matrix4x4 Transform::LocalRotation4x4()
{
    float matrix[4][4] =
    {
        {this->localRotation.m[0][0], this->localRotation.m[0][1], this->localRotation.m[0][2], 0},
        {this->localRotation.m[1][0], this->localRotation.m[1][1], this->localRotation.m[1][2], 0},
        {this->localRotation.m[2][0], this->localRotation.m[2][1], this->localRotation.m[2][2], 0},
        {0, 0, 0, 1}
    };

    return Matrix4x4(matrix);
}

Matrix4x4 Transform::LocalTranslation4x4()
{
    float matrix[4][4] =
    {
        {1, 0, 0, this->localPosition.x},
        {0, 1, 0, this->localPosition.y},
        {0, 0, 1, this->localPosition.z},
        {0, 0, 0, 1}
    };

    return Matrix4x4(matrix);
}

Matrix4x4 Transform::LocalTranslation4x4Inverse()
{
    float matrix[4][4] =
    {
        {1, 0, 0, -this->localPosition.x},
        {0, 1, 0, -this->localPosition.y},
        {0, 0, 1, -this->localPosition.z},
        {0, 0, 0, 1}
    };

    return Matrix4x4(matrix);
}

// 1:Scale, 2:Rotate, 3:Translate
Matrix4x4 Transform::TRS()
{
    float trs[4][4] =
    {
        {this->localRotation.m[0][0] * localScale.x, this->localRotation.m[0][1] * localScale.y, this->localRotation.m[0][2] * localScale.z, localPosition.x},
        {this->localRotation.m[1][0] * localScale.x, this->localRotation.m[1][1] * localScale.y, this->localRotation.m[1][2] * localScale.z, localPosition.y},
        {this->localRotation.m[2][0] * localScale.x, this->localRotation.m[2][1] * localScale.y, this->localRotation.m[2][2] * localScale.z, localPosition.z},
        {0,                                         0,                                  0,                      1}
    };

    if (parent) {
        return parent->TRS() * trs;
    }

    return trs;
}

// S^-1 * R^-1 * T^-1
Matrix4x4 Transform::TRSInverse()
{
    if (parent) {
        return LocalScale4x4Inverse() * Matrix4x4::Transpose(LocalRotation4x4()) * LocalTranslation4x4Inverse() * parent->TRSInverse();
    }

    return LocalScale4x4Inverse() * Matrix4x4::Transpose(LocalRotation4x4()) * LocalTranslation4x4Inverse();
}

// 1:Rotate, 2:Translate
Matrix4x4 Transform::TR()
{
    float tr[4][4] =
    {
        {this->localRotation.m[0][0], this->localRotation.m[0][1], this->localRotation.m[0][2], localPosition.x},
        {this->localRotation.m[1][0], this->localRotation.m[1][1], this->localRotation.m[1][2], localPosition.y},
        {this->localRotation.m[2][0], this->localRotation.m[2][1], this->localRotation.m[2][2], localPosition.z},
        {0,                                         0,                                  0,                      1}
    };
    if (parent) {
        return parent->TR() * tr;
    }

    return tr;
}

// R^-1 * T^-1
Matrix4x4 Transform::TRInverse()
{
    if (parent) {
        return  Matrix4x4::Transpose(LocalRotation4x4()) * LocalTranslation4x4Inverse() * parent->TRInverse();
    }

    return Matrix4x4::Transpose(LocalRotation4x4()) * LocalTranslation4x4Inverse();
}

//-----------------------------CAMERA-------------------------------------------------
struct CameraSettings
{
    static bool outsiderViewPerspective;
    static bool displayReticle;
};
bool CameraSettings::outsiderViewPerspective = false;
bool CameraSettings::displayReticle = true;

class Camera : public Transform
{
    Mesh* mesh;
public:
    static Camera* main;
    static Camera* projector;
    static List<Camera*> cameras;
    static int cameraCount;

    void SetMesh(Mesh* mesh)
    {
        this->mesh = mesh;
        this->mesh->SetParent(this, false);
        this->mesh->localPosition += -Direction::forward;
    }

    Mesh* GetMesh()
    {
        return this->mesh;
    }
    
    std::string name;

    Camera(const const Vec3& position = Vec3(0, 0, 0), const Vec3& rotationEuler = Vec3(0, 0, 0))
        : Transform(1, position, rotationEuler)
    {
        cameras.emplace(cameras.begin() + cameraCount++, this);
        this->name = "Camera " + cameraCount;
    }
    
    static bool InsideViewScreen(Vec3* verts_proj, int size)
    {
        Range xRange = ProjectVertsOntoAxis(verts_proj, size, Direction::right);
        if ((xRange.min > 1.0f || xRange.max < -1.0f)) {
            return false;
        }
        Range yRange = ProjectVertsOntoAxis(verts_proj, size, Direction::up);
        if ((yRange.min > 1.0f || yRange.max < -1.0f)) {
            return false;
        }

        return true;
    }
};
List<Camera*> Camera::cameras = List<Camera*>();
int Camera::cameraCount = 0;
Camera* Camera::projector = new Camera();
Camera* camera1 = new Camera();
Camera* camera2 = new Camera(Vec3(0, 50, 0), Vec3(-90 * PI / 180, 0, 0));
Camera* Camera::main = camera1;

//---------------------------------MESH---------------------------------------------

bool Mesh::SetVisibility(bool visible)
{
    if (visible) {
        ManagedObjectPool<Mesh>::AddToPool(this);
        return true;
    }
    else {
        ManagedObjectPool<Mesh>::RemoveFromPool(this);
        return false;
    }
}
void Mesh::SetColor(Color&& c) {
    SetColor(c);
}

void Mesh::SetColor(Color& c)
{
    color = c;
    if (vertices.size() > 0)
    {
        for (int i = 0; i < triangles->size(); i++)
        {
            (*triangles)[i].color = c;
        }
    }
}

List<Triangle>* Mesh::MapVertsToTriangles()
{
    if (indices && !vertices.empty())
    {
        int t = 0;
        for (size_t i = 0; i < indices->size(); i++)
        {
            int p1Index = (*indices)[i++];
            int p2Index = (*indices)[i++];
            int p3Index = (*indices)[i];

            (*triangles)[t].verts[0] = (vertices)[p1Index];
            (*triangles)[t].verts[1] = (vertices)[p2Index];
            (*triangles)[t].verts[2] = (vertices)[p3Index];
            
            t++;
        }
    }

    return triangles;
}

//Convert to world coordinates
List<Vec3> Mesh::WorldVertices()
{
    List<Vec3> verts = vertices;
    Matrix4x4 matrix = TRS();
    for (size_t i = 0; i < verts.size(); i++)
    {
        verts[i] = (Vec3)(matrix * ((Vec4)verts[i]));
    }

    return verts;
}

void Mesh::TransformTriangles()
{
    // Scale/Distance ratio culling
    /* bool tooSmallToSee = scale.SqrMagnitude() / (position - Camera::main->position).SqrMagnitude() < 0.000000125;
    if (tooSmallToSee) {
        return;
    }*/

    Matrix4x4 modelToWorldMatrix = this->TRS();

    //Transform Triangles
    List<Triangle>* tris = MapVertsToTriangles();

    for (int i = 0; i < tris->size(); i++)
    {
        Triangle tri = (*tris)[i];
        tri.mesh = this;
        Triangle worldSpaceTri = tri;
        Triangle camSpaceTri = tri;
        Triangle projectedTri = tri;
        for (int j = 0; j < 3; j++)
        {
            // Homogeneous coords (x, y, z, w=1)
            Vec4 vert = tri.verts[j];

            // =================== WORLD SPACE ===================
            // Transform local coords to world-space coords.
            Vec4 worldPoint = modelToWorldMatrix * vert;
            worldSpaceTri.verts[j] = worldPoint;

            // ================ VIEW/CAM/EYE SPACE ================
            // Transform world coordinates to view coordinates.
            Vec4 cameraSpacePoint = worldToViewMatrix * worldPoint;
            camSpaceTri.verts[j] = cameraSpacePoint;

            // ================ SCREEN SPACE ==================
            // Project to screen space (image space)
            Vec4 projectedPoint = projectionMatrix * cameraSpacePoint;
            projectedTri.verts[j] = projectedPoint;
        };

        //------------------- Normal/Frustum Culling (view space)------------------------
        Vec3 p1_c = camSpaceTri.verts[0];
        Vec3 p2_c = camSpaceTri.verts[1];
        Vec3 p3_c = camSpaceTri.verts[2];
        camSpaceTri.Centroid();

        if (Graphics::frustumCulling)
        {
            bool tooCloseToCamera = (p1_c.z >= nearClippingPlane || p2_c.z >= nearClippingPlane || p3_c.z >= nearClippingPlane || camSpaceTri.centroid.z >= nearClippingPlane);
            if (tooCloseToCamera) {
                continue;
            }

            bool tooFarFromCamera = (p1_c.z <= farClippingPlane || p2_c.z <= farClippingPlane || p3_c.z <= farClippingPlane || camSpaceTri.centroid.z <= farClippingPlane);
            if (tooFarFromCamera) {
                continue;
            }
            /*
            bool behindCamera = DotProduct((Vec3)camSpaceTri.centroid, Direction::forward) <= 0.0;
            if (behindCamera) {
                continue; // Skip triangle if it's out of cam view.
            }*/
            if (!Camera::InsideViewScreen(projectedTri.verts, 3)) {
                continue;
            }
        }

        // Calculate triangle suface Normal
        camSpaceTri.Normal();//camSpaceTri.normal = worldToViewMatrix * modelToWorldMatrix * Vec4(camSpaceTri.normal, 0);

        if (Graphics::invertNormals) {
            camSpaceTri.normal = ((Vec3)camSpaceTri.normal) * -1.0f;
        }

        // Back-face Culling - Checks if the triangles backside is facing the camera.
        // Condition makes this setting optional when drawing wireframes alone, but will force culling if triangles are filled.
        if (Graphics::backFaceCulling || Graphics::fillTriangles)
        {
            Vec3 posRelativeToCam = camSpaceTri.centroid;// Since camera is (0,0,0) in view space, the displacement vector from camera to centroid IS the centroid itself.
            bool faceInvisibleToCamera = DotProduct(posRelativeToCam, (Vec3)camSpaceTri.normal) >= 0;
            if (faceInvisibleToCamera) {
                continue;// Skip triangle if it's out of cam view or it's part of the other side of the mesh.
            }
        }

        //------------------------ Lighting (world space)------------------------

        if (Graphics::lighting && Graphics::fillTriangles)
        {
            if (!ignoreLighting)
            {
                float amountFacingLight = DotProduct((Vec3)worldSpaceTri.Normal(), lightSource);
                Color colorLit = projectedTri.color * Clamp(amountFacingLight, 0.15, 1);
                projectedTri.color = colorLit;
            }
        }

        if (Graphics::vfx)
        {
            Vec3 screenLeftSide = Vec3(-1, 0, 0);
            Vec3 screenRightSide = Vec3(1, 0, 0);
            Range range = ProjectVertsOntoAxis(projectedTri.verts, 3, screenRightSide);
            bool rightHalfScreenX = range.min > 0;// && range.max < 1;

            if (rightHalfScreenX) {
                projectedTri.color = Color(0, 0, 255);// std::cout << "Inside" << std::endl;
            }
            else {
                projectedTri.color = Color::red;
            }
        }
        // ---------- Debugging -----------
        if (Graphics::debugNormals)
        {
            //---------Draw point at centroid and a line from centroid to normal (view space & projected space)-----------
            float normalScalar = (0.011f * ((Vec3)camSpaceTri.centroid).Magnitude());//Scaled depending on distance from camera
            Vec2 centroidToNormal_p = projectionMatrix * ((Vec3)camSpaceTri.centroid + camSpaceTri.normal * normalScalar);
            Vec2 centroid_p = projectionMatrix * camSpaceTri.centroid;
            Point::AddPoint(Point(centroid_p));
            Line::AddLine(Line(centroid_p, centroidToNormal_p));
        }

        projectedTri.centroid = projectionMatrix * camSpaceTri.centroid;

        // Nested Projection or Double Projection
        if (CameraSettings::outsiderViewPerspective)
        {
            Matrix4x4 nestedProjectionMatrix = weakPerspectiveProjectionMatrix * Camera::projector->TRInverse();

            for (size_t k = 0; k < 3; k++)
            {
                projectedTri.verts[k] = nestedProjectionMatrix * projectedTri.verts[k];
            }
        }

        //Add projected tri
        triBuffer->emplace_back(projectedTri);
    }
}
//List<Mesh*> Mesh::objects = List<Mesh*>(1000);
//int Mesh::meshCount = 0;
int Mesh::worldTriangleDrawCount = 0;

//------------------------------------CUBE MESH------------------------------------------
class CubeMesh : public Mesh
{
public:
    CubeMesh(const float& scale = 1, const Vec3& position = Vec3(0, 0, 0), const Vec3& rotationEuler = Vec3(0, 0, 0))
        :Mesh(scale, position, rotationEuler)
    {
        // Local Space (Object Space)
        this->vertices = List<Vec3>({//new Vec3[8] {
            //south
            Vec3(-0.5, -0.5, 0.5),
            Vec3(-0.5, 0.5, 0.5),
            Vec3(0.5, 0.5, 0.5),
            Vec3(0.5, -0.5, 0.5),
            //north
            Vec3(-0.5, -0.5, -0.5),
            Vec3(-0.5, 0.5, -0.5),
            Vec3(0.5, 0.5, -0.5),
            Vec3(0.5, -0.5, -0.5)
            });

        this->indices = new List<int>{
            //South
            0, 1, 2,
            0, 2, 3,
            //North
            7, 6, 5,
            7, 5, 4,
            //Right
            3, 2, 6,
            3, 6, 7,
            //Left
            4, 5, 1,
            4, 1, 0,
            //Top
            1, 5, 6,
            1, 6, 2,
            //Bottom
            3, 7, 4,
            3, 4, 0
        };

        triangles = new List<Triangle>(this->indices->size() / 3);

        bounds->CreateBounds(this);
    }
};

//---------------------------------PLANE---------------------------------------------
class PlaneMesh : public Mesh
{
public:
    PlaneMesh(const float& scale = 1, const Vec3& position = Vec3(0, 0, 0), const Vec3& rotationEuler = Vec3(0, 0, 0))
        :Mesh(scale, position, rotationEuler)
    {
        this->vertices = List<Vec3>{
                Vec3(-0.5, 0, 0.5),
                Vec3(-0.5, 0, -0.5),
                Vec3(0.5, 0, -0.5),
                Vec3(0.5, 0, 0.5)
        };

        this->triangles->emplace_back(Triangle((vertices)[0], (vertices)[1], (vertices)[2]));
        this->triangles->emplace_back(Triangle((vertices)[0], (vertices)[2], (vertices)[3]));
        
        bounds->CreateBounds(this);
    }
};

void BoundingBox::CreateBounds(Mesh* mesh)
{
    if (!mesh || mesh->vertices.size() < 1)
    {
        return;
    }

    this->mesh = mesh;

    Range xRange = ProjectVertsOntoAxis(mesh->vertices.data(), mesh->vertices.size(), Direction::right);
    Range yRange = ProjectVertsOntoAxis(mesh->vertices.data(), mesh->vertices.size(), Direction::up);
    Range zRange = ProjectVertsOntoAxis(mesh->vertices.data(), mesh->vertices.size(), Direction::back);

    min = Vec3(xRange.min, yRange.min, zRange.min);
    max = Vec3(xRange.max, yRange.max, zRange.max);
    
    bounds = Cube(min, max);
}

List<Vec3>* BoundingBox::WorldVertices()
{
    if (!this->mesh)
    {
        return nullptr;
    }

    static List<Vec3> verts = List<Vec3>(8);

    auto trs4x4 = mesh->TRS();
    for (size_t i = 0; i < 8; i++)
    {
        verts[i] = trs4x4 * bounds.vertices[i];
    }

    return &verts;
}

List<Vec3>* BoundingBox::ViewspaceVertices()
{
    if (!this->mesh)
    {
        return nullptr;
    }

    static List<Vec3> verts = List<Vec3>(8);

    auto mat4x4 = Camera::main->TRInverse() * mesh->TRS();
    for (size_t i = 0; i < 8; i++)
    {
        verts[i] = mat4x4 * bounds.vertices[i];
    }

    return &verts;
}

List<Vec3>* BoundingBox::ProjectedVertices()
{
    if (!this->mesh)
    {
        return nullptr;
    }

    static List<Vec3> verts = List<Vec3>(8);

    auto mat4x4 = projectionMatrix * Camera::main->TRInverse() * mesh->TRS();
    for (size_t i = 0; i < 8; i++)
    {
        verts[i] = mat4x4 * bounds.vertices[i];
    }

    return &verts;
}

void BoundingBox::Draw()
{
    if (this->mesh)
    {
        auto vertices_w = WorldVertices();
        //Point::AddWorldPoint(Point(mesh->TRS() * min, Color::orange, 10));
        //Point::AddWorldPoint(Point(mesh->TRS() * max, Color::yellow, 10));
        Line::AddWorldLine(Line((*vertices_w)[0], (*vertices_w)[1], color));
        Line::AddWorldLine(Line((*vertices_w)[1], (*vertices_w)[2], color));
        Line::AddWorldLine(Line((*vertices_w)[2], (*vertices_w)[3], color));
        Line::AddWorldLine(Line((*vertices_w)[3], (*vertices_w)[0], color));
        Line::AddWorldLine(Line((*vertices_w)[4], (*vertices_w)[5], color));
        Line::AddWorldLine(Line((*vertices_w)[5], (*vertices_w)[6], color));
        Line::AddWorldLine(Line((*vertices_w)[6], (*vertices_w)[7], color));
        Line::AddWorldLine(Line((*vertices_w)[7], (*vertices_w)[4], color));
        Line::AddWorldLine(Line((*vertices_w)[0], (*vertices_w)[4], color));
        Line::AddWorldLine(Line((*vertices_w)[1], (*vertices_w)[5], color));
        Line::AddWorldLine(Line((*vertices_w)[2], (*vertices_w)[6], color));
        Line::AddWorldLine(Line((*vertices_w)[3], (*vertices_w)[7], color));
    }
}

//------------------------------HELPER FUNCTIONS------------------------------------------------

Mesh* LoadMeshFromOBJFile(std::string objFileName)
{
    static std::string filePath = "./Objects/";

    std::string mtlFileName = "";
    std::ifstream mtlFile;

    // ----------- Read object file -------------
    List<std::string> strings;
    std::string line;
    std::ifstream objFile;
    objFile.open(filePath + objFileName);
    if (objFile.is_open())
    {
        while (objFile) {
            // 1st. Gets the next line.
            // 2nd. Seperates each word from that line then stores each word into the std::strings array.
            getline(objFile, line);
            std::string word;
            std::stringstream ss(line);
            while (getline(ss, word, ' '))
            {
                if (word == "mtllib") {
                    getline(ss, word, ' ');
                    mtlFileName = word;
                }
                else {
                    strings.emplace_back(word);
                }
            }
        }
    }
    objFile.close();

    // -----------------Construct new mesh-------------------
    
    List<Vec3> verts = List<Vec3>();
    List<int>* indices = new List<int>();
    List<Triangle>* triangles = new List<Triangle>(indices->size() / 3);
    Material material;
    for (size_t i = 0; i < strings.size(); i++)
    {
        // v = vertex
        // f = face
        std::string objFileSubString = strings[i];

        // if .obj encounters "usemtl" then the next string will be material id.
        if (objFileSubString == "usemtl")
        {
            // Try opening Material file to see if it exists
            mtlFile.open(filePath + mtlFileName);
            // check if using material before looking for material key words
            if (mtlFile.is_open())
            {
                std::string mtlID = strings[++i];
                bool mtlIDFound = false;
                //materials->emplace_back(Material(materialID));
                // search .mtl for material properties under the the current materialID
                while (!mtlIDFound && mtlFile)
                {
                    // 1st. Gets the next line.
                    // 2nd. Seperates each word from that line then stores each word into the strings array.
                    getline(mtlFile, line);
                    std::string word;
                    std::stringstream ss(line);
                    while (!mtlIDFound && getline(ss, word, ' '))
                    {
                        if (word == "newmtl")
                        {
                            getline(ss, word, ' ');
                            if (mtlID == word) {
                                material.name = word;
                            }
                        }
                        else if (mtlID == material.name && word == "Kd") {
                            getline(ss, word, ' ');
                            float r = stof(word);
                            getline(ss, word, ' ');
                            float g = stof(word);
                            getline(ss, word, ' ');
                            float b = stof(word);
                            material.color = Color(255 * r, 255 * g, 255 * b);

                            mtlIDFound = true;
                        }
                    }
                }
                mtlFile.close();
            }
        }

        else if (objFileSubString == "v") {
            float x = stof(strings[++i]);
            float y = stof(strings[++i]);
            float z = stof(strings[++i]);
            verts.emplace_back(Vec3(x, y, z));
        }
        //f means the next 3 strings will be the indices for mapping vertices
        else if (objFileSubString == "f") {
            int p3Index = stof(strings[++i]) - 1;
            int p2Index = stof(strings[++i]) - 1;
            int p1Index = stof(strings[++i]) - 1;

            indices->emplace_back(p1Index);
            indices->emplace_back(p2Index);
            indices->emplace_back(p3Index);

            Triangle tri = Triangle((verts)[p1Index], (verts)[p2Index], (verts)[p3Index]);
            tri.color = material.color;
            triangles->emplace_back(tri);
        }
    }
    //mtlFile.close();
    Mesh* mesh = new Mesh();
    mesh->vertices = verts;
    mesh->indices = indices;
    mesh->triangles = triangles;
    mesh->bounds->CreateBounds(mesh);

    return mesh;
}

#include <OctTree.h>

void Draw()
{
    // Camera TRInverse = (TR)^-1 = R^-1*T^-1 = M = Mcw = World to Camera coords. 
    // This matrix isn't used on the camera itself, but we record the reverse transformations of the camera going from world space back to local camera space.
    // Every point then in world space multiplied by this matrix will end up in a position relative to the camera's point of view when it was in world space. 
    // The camera could now be considered as the origin (0,0,0) with the zero rotation (identity matrix). 
    worldToViewMatrix = Camera::main->TRInverse();
    projectionMatrix = ProjectionMatrix();
    Matrix4x4 vpMatrix = projectionMatrix * worldToViewMatrix;

    /*
    int nodeCount = 0;
    auto visible = [&](TreeNode<Mesh>* node) mutable {
        List<Vec3>* verts = node->bounds->WorldVertices();
        Vec3 camPos = Camera::main->Position();
        Vec3 camLooking = Camera::main->Forward();
        for (size_t v = 0; v < verts->size(); v++)
        {
            bool behindCamera = DotProduct((*verts)[v] - camPos, camLooking) < 0;
            if (!behindCamera)
            {
                nodeCount++;
                return true;
            }
        }
        return false;
    };
    
    List<Mesh*> meshes;
    OctTree<Mesh>::Update();
    OctTree<Mesh>::Tree()->Extract(meshes);
    for (size_t i = 0; i < 8; i++)
    {
        auto zone = OctTree<Mesh>::Tree()->children->at(i);
        if (visible(zone))
        {
            zone->Extract(meshes);
            zone->ForEachSubNode([&](TreeNode<Mesh>* sub) {
                if (visible(sub))
                {
                    sub->Extract(meshes);
                }
            });
        }
    }

    if (DEBUGGING) {
        std::cout << "Nodes: " << OctTree<Mesh>::count << std::endl;
        std::cout << "Nodes Visible: " << nodeCount << std::endl;
        std::cout << "Meshes Looping: " << meshes.size() << std::endl;
    }
    */
    // ---------- Transform -----------
    for (int i = 0; i < Mesh::count; i++)
    {
        Mesh* mesh = Mesh::objects[i];
        BoundingBox* bounds = mesh->bounds;

        if (Graphics::frustumCulling)
        {
            // Scale/Distance ratio culling
            /*float sqrDist = (mesh->root->localPosition - Camera::main->Position()).SqrMagnitude();
            if (sqrDist != 0.0)
            {
                bool meshTooSmallToSee = mesh->root->localScale.SqrMagnitude() / sqrDist < 0.0000000000001;
                if (meshTooSmallToSee) {
                    continue;
                }
            }*/
            /*
            bool meshBehindCamera = DotProduct((Mesh::objects[i]->Position() - Camera::main->position), Camera::main->Forward()) <= 0.0;
            if (meshBehindCamera) {
                continue;
            }
*/
            if (bounds)
            {
                // Check if behind camera
                List<Vec3> verts_v = *bounds->ViewspaceVertices();
                Range range = ProjectVertsOntoAxis(verts_v.data(), verts_v.size(), Direction::forward);
                if (range.max < 0)
                {
                    continue;
                }

                // Check if visible
                List<Vec3>* verts_p = bounds->ProjectedVertices();
                if (!Camera::InsideViewScreen(verts_p->data(), 8))
                {
                    continue;
                }
            }
        }

        if (bounds)
        {
            if (Graphics::debugBounds)
            {
                if (mesh != Camera::main->GetMesh() && DotProduct(mesh->Position() - Camera::main->Position(), Camera::main->Forward()) > 0)
                {
                    bounds->Draw();
                }
            }
        }

        if (Graphics::debugAxes)
        {
            if (DotProduct(mesh->Position() - Camera::main->Position(), Camera::main->Forward()) > 0)
            {
                Matrix4x4 mvp = vpMatrix * mesh->TRS();

                Vec2 center_p = mvp * Vec4(0, 0, 0, 1);
                Vec2 xAxis_p = mvp * Vec4(0.5, 0, 0, 1); 
                Vec2 yAxis_p = mvp * Vec4(0, 0.5, 0, 1);
                Vec2 zAxis_p = mvp * Vec4(0, 0, 0.5, 1);
                Vec2 forward_p = mvp * (Direction::forward);

                Point::AddPoint(Point(center_p, Color::red, 4));
                Line::AddLine(Line(center_p, xAxis_p, Color::red));
                Line::AddLine(Line(center_p, yAxis_p, Color::yellow));
                Line::AddLine(Line(center_p, zAxis_p, Color::blue));
                Line::AddLine(Line(center_p, mvp * (Direction::forward), Color::turquoise, 3));
            }
        }

        mesh->TransformTriangles();
    }

    Mesh::worldTriangleDrawCount = triBuffer->size();

    // ---------- Sort (Painter's algorithm) -----------
    sort(triBuffer->begin(), triBuffer->end(), [](const Triangle& triA, const Triangle& triB) -> bool {
        return triA.centroid.w > triB.centroid.w;
        });
    /*
    Matrix4x4 matrix = ProjectionMatrix() * Camera::main->TRInverse();

    Vec3 corner1 = ((Direction::left + Direction::down) * 0.2);
    Vec3 corner2 = ((Direction::down) * 0.5);
    Vec3 corner3 = ((Direction::right + Direction::down) * 0.2);
    Vec3 corner4 = ((Direction::right) * 0.2);
    Vec3 corner5 = ((Direction::right + Direction::up) * 0.2);
    Vec3 corner6 = ((Direction::up) * 0.5);
    Vec3 corner7 = ((Direction::left + Direction::up) * 0.2);
    Vec3 corner8 = ((Direction::left) * 0.2);

    Plane plane1 = Plane(Camera::main->TRS()*corner1, Camera::main->TRS() * corner2, Camera::main->TRS() * Vec3(0,0,1));
    lineBuffer->emplace_back(Line(corner1, corner2, Color::purple, 4));
    lineBuffer->emplace_back(Line(corner2, corner3, Color::green, 4));
    lineBuffer->emplace_back(Line(corner3, corner4, Color::green, 4));
    lineBuffer->emplace_back(Line(corner4, corner5, Color::green, 4));
    lineBuffer->emplace_back(Line(corner5, corner6, Color::green, 4));
    lineBuffer->emplace_back(Line(corner6, corner7, Color::green, 4));
    lineBuffer->emplace_back(Line(corner7, corner8, Color::green, 4));
    lineBuffer->emplace_back(Line(corner8, corner1, Color::green, 4));

    lineBuffer->emplace_back(Line(ProjectionMatrix()*corner1, ProjectionMatrix() * corner2, Color::pink, 4));
    lineBuffer->emplace_back(Line(ProjectionMatrix() * corner2, ProjectionMatrix() * corner3, Color::blue, 4));
    lineBuffer->emplace_back(Line(ProjectionMatrix() * corner3, ProjectionMatrix() * corner4, Color::blue, 4));
    lineBuffer->emplace_back(Line(ProjectionMatrix() * corner4, ProjectionMatrix() * corner5, Color::blue, 4));
    lineBuffer->emplace_back(Line(ProjectionMatrix() * corner5, ProjectionMatrix() * corner6, Color::blue, 4));
    lineBuffer->emplace_back(Line(ProjectionMatrix() * corner6, ProjectionMatrix() * corner7, Color::blue, 4));
    lineBuffer->emplace_back(Line(ProjectionMatrix() * corner7, ProjectionMatrix() * corner8, Color::blue, 4));
    lineBuffer->emplace_back(Line(ProjectionMatrix() * corner8, ProjectionMatrix() * corner1, Color::blue, 4));


    for (int i = -50; i < 50; i++)
    {
        Vec3 from = Vec3(i, 0, -100);
        Vec3 to = Vec3(i, 0, 100);
        Vec3 from_c = Camera::main->TRInverse() * from;
        Vec3 to_c = Camera::main->TRInverse()* to;
        Vec3 from_p = matrix * from;
        Vec3 to_p = matrix * to;


        Vec3 intersection = Vec3();
        if (LinePlaneIntersecting(from, to, plane1, &intersection))
        {
            Point::AddWorldPoint(Point(intersection, Color::purple, 10));
            Point::AddPoint(Point(Camera::main->TRInverse()* intersection, Color::orange, 8));

            Vec3 n_c = Camera::main->TRInverse()*plane1.Normal();
            bool fromIsInside = DotProduct(from_c, n_c) > 0;
            bool toIsInside = DotProduct(to_c, n_c) > 0;
            if (!fromIsInside)
            {
                from = intersection;
            }
            else if (!toIsInside)
            {
                to = intersection;
            }

            Line::AddWorldLine(Line(from, to, Color::white, 4));
            Point::AddWorldPoint(Point(from, Color::orange, 10));
            Point::AddWorldPoint(Point(to, Color::yellow, 10));

        }
        else {
            bool p1Outside = (from_p.x > 0.5 || from_p.x < -0.5) || (from_p.y > 0.5 || from_p.y < -0.5);
            bool p2Outside = (to_p.x > 0.5 || to_p.x < -0.5) || (to_p.y > 0.5 || to_p.y < -0.5);

            from_p.x = Clamp(from_p.x, -0.5, 0.5);
            from_p.y = Clamp(from_p.y, -0.5, 0.5);
            to_p.x = Clamp(to_p.x, -0.5, 0.5);
            to_p.y = Clamp(to_p.y, -0.5, 0.5);

            Point::AddPoint(Point(from, Color::red, 4));
            Point::AddPoint(Point(to, Color::red, 4));
            Line::AddWorldLine(Line(from, to, Color::white, 4));
        }
    }
    //--------------------------------------------------------------------------------
    Matrix4x4 matrix = ProjectionMatrix() * Camera::main->TRInverse();

    Vec3 corner1 = ((Direction::left + Direction::down) * 0.5);
    Vec3 corner2 = ((Direction::right + Direction::down) * 0.5);
    Vec3 corner3 = ((Direction::right + Direction::up) * 0.5);
    Vec3 corner4 = ((Direction::left + Direction::up) * 0.5);

    lineBuffer->emplace_back(Line(corner1, corner2, Color::green, 4));
    lineBuffer->emplace_back(Line(corner2, corner3, Color::green, 4));
    lineBuffer->emplace_back(Line(corner3, corner4, Color::green, 4));
    lineBuffer->emplace_back(Line(corner4, corner1, Color::green, 4));


    for (int i = -50; i < 50; i++)
    {
        Vec3 from = Vec3(i, 0, -100);
        Vec3 to = Vec3(i, 0, 100);

            Vec3 from_p = matrix * from;
            Vec3 to_p = matrix * to;

            //bool p1Outside = (from_p.x > 0.5 || from_p.x < -0.5) || (from_p.y > 0.5 || from_p.y < -0.5);
            //bool p2Outside = (to_p.x > 0.5 || to_p.x < -0.5) || (to_p.y > 0.5 || to_p.y < -0.5);

            from_p.x = Clamp(from_p.x, -0.5, 0.5);
            from_p.y = Clamp(from_p.y, -0.5, 0.5);
            to_p.x = Clamp(to_p.x, -0.5, 0.5);
            to_p.y = Clamp(to_p.y, -0.5, 0.5);

            pointBuffer->emplace_back(Point(from_p, Color::red, 4));
            pointBuffer->emplace_back(Point(to_p, Color::red, 4));
            Line::AddLine(Line(from_p, to_p, Color::green, 4));
    }*/
    //---------------------------------------------------------------------------------------------------*/

    // ---------- Draw -----------
    for (int i = 0; i < triBuffer->size(); i++)
    {
        (*triBuffer)[i].Draw();
    }

    for (size_t i = 0; i < lineBuffer->size(); i++)
    {
        (*lineBuffer)[i].Draw();
    }

    for (size_t i = 0; i < pointBuffer->size(); i++)
    {
        (*pointBuffer)[i].Draw();
    }

    pointBuffer->clear();
    lineBuffer->clear();
    triBuffer->clear();
}
#endif