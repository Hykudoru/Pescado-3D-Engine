#pragma once
#include <Matrix.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#ifndef GRAPHICS_H
#define GRAPHICS_H

#define Color Vector3<float>
struct RGB : Vector3<float>
{
    static Color black;
    static Color white;
    static Color red;
    static Color green;
    static Color blue;
    static Color pink;
    static Color yellow;
    static Color turquoise;
    static Color orange;
};
Color RGB::black = Color(0, 0, 0);
Color RGB::white = Color(255, 255, 255);
Color RGB::red = Color(255, 0, 0);
Color RGB::green = Color(0, 255, 0);
Color RGB::blue = Color(0, 0, 255);
Color RGB::pink = Color(255, 0, 255);
Color RGB::yellow = Color(255, 255, 0);
Color RGB::turquoise = Color(0, 255, 255);
Color RGB::orange = Color(255, 158, 0);

#define List std::vector
const float PI = 3.14159265359f;

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

float ToDeg(float rad) {
    return rad * 180.0 / PI;
}

float ToRad(float deg) {
    return deg * PI / 180.0;
}

int screenWidth = 800;//700;//screen.width - 20;
int screenHeight = 800; //screen.height - 20; //screen.height;// - 30;

//-----------GRAPHICS---------------

struct GraphicSettings
{
    static bool culling;
    static bool invertNormals;
    static bool debugNormals;
    static bool debugVertices;
    static bool perspective;
    static bool fillTriangles;
    static bool displayWireFrames;
    static bool lighting;
    static bool vfx;
};
bool GraphicSettings::culling = true;
bool GraphicSettings::invertNormals = false;
bool GraphicSettings::debugNormals = false;
bool GraphicSettings::debugVertices = false;
bool GraphicSettings::perspective = true;
bool GraphicSettings::fillTriangles = true;
bool GraphicSettings::displayWireFrames = false;
bool GraphicSettings::lighting = true;
bool GraphicSettings::vfx = false;

static float worldScale = 1;

struct Vec3D
{
    static Vec3 forward;
    static Vec3 back;
    static Vec3 right;
    static Vec3 left;
    static Vec3 up;
    static Vec3 down;
};
Vec3 Vec3D::forward = Vec3(0, 0, -1);
Vec3 Vec3D::back = Vec3(0, 0, 1);
Vec3 Vec3D::right = Vec3(1, 0, 0);
Vec3 Vec3D::left = Vec3(-1, 0, 0);
Vec3 Vec3D::up = Vec3(0, 1, 0);
Vec3 Vec3D::down = Vec3(0, -1, 0);

float aspectRatio = screenWidth / screenHeight;
float nearClippingPlane = -0.1;
float farClippingPlane = -10000.0;
float fieldOfViewDeg = 60;
float fov = ToRad(fieldOfViewDeg);

// Perspective Projection Matrix
float persp[4][4] = {
    {1 / tan(fov / 2), 0, 0, 0},
    {0, 1 / tan(fov / 2), 0, 0},
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

Matrix4x4 perspectiveProjectionMatrix = persp;
Matrix4x4 orthographicProjectionMatrix = ortho;

void FOV(int deg)
{
    fieldOfViewDeg = deg;
    fov = ToRad(deg);
    float newPerspectiveProjectionMatrix[4][4] = {
        {1/tan(fov/2), 0, 0, 0},
        {0, 1/tan(fov/2), 0, 0},
        {0, 0, 1, 0},
        {0, 0, -1, 0}
    };

    perspectiveProjectionMatrix = newPerspectiveProjectionMatrix;
}

void Point(float x, float y)
{
    glBegin(GL_POINTS);
    glVertex2f(x, y);
    glEnd();
}

void Line(float x0, float y0, float x, float y)
{
    glBegin(GL_LINES);
    glVertex2f(x0, y0);
    glVertex2f(x, y);
    glEnd();
}

Matrix4x4 ProjectionMatrix()
{
    if (GraphicSettings::perspective) {
        return perspectiveProjectionMatrix;
    }
    else {
        return orthographicProjectionMatrix;
    }
}

Vec4 ProjectPoint(Vec4 point)
{
    point = ProjectionMatrix() * point;

    point.x *= worldScale;
    point.y *= worldScale;
    return point;
}

struct Triangle
{
    Vec3 verts[3];
    Color color;
    Vec4 centroid;
    Vec3 normal;

    Triangle()
    {
        color = RGB::white;
        centroid = Vec3(0, 0, 0);
        normal = Vec3(0, 0, 0);
    };

    Triangle(Vec3 p1, Vec3 p2, Vec3 p3)
    {
        this->verts[0] = p1;
        this->verts[1] = p2;
        this->verts[2] = p3;
        color = RGB::white;
        centroid = Vec3(0, 0, 0);
        normal = Vec3(0, 0, 0);
    }

    Vec4 Normal()
    {
        // Calculate triangle suface Normal
        Vec3 a = verts[2] - verts[0];
        Vec3 b = verts[1] - verts[0];
        Vec3 normal = (CrossProduct(a, b)).Normalized();

        return normal;
    }

    Vec4 Centroid()
    {
        Vec4 centroid = Vec4(
            (verts[0].x + verts[1].x + verts[2].x) / 3.0,
            (verts[0].y + verts[1].y + verts[2].y) / 3.0,
            (verts[0].z + verts[1].z + verts[2].z) / 3.0
        );

        return centroid;
    }

    static void DrawTriangle(Triangle tri)
    {
        Vec4 p1 = tri.verts[0];
        Vec4 p2 = tri.verts[1];
        Vec4 p3 = tri.verts[2];

        if (GraphicSettings::debugVertices) 
        {
            float c = Clamp(1.0 / (0.000001 + (tri.color.x + tri.color.y + tri.color.z) / 3), 0, 255);
            glColor3ub(c, c, c);

            Point(p1.x, p1.y);
            Point(p2.x, p2.y);

            Point(p2.x, p2.y);
            Point(p3.x, p3.y);

            Point(p3.x, p3.y);
            Point(p1.x, p1.y);
        }

        if (GraphicSettings::fillTriangles == false) 
        {
            GraphicSettings::displayWireFrames = true;
        }

        glColor3ub(255, 255, 255);

        if (GraphicSettings::displayWireFrames)
        {
            glBegin(GL_LINES);
            if (GraphicSettings::fillTriangles)
            {
                float c = Clamp(1.0 / (0.000001 + (tri.color.x + tri.color.y + tri.color.z) / 3), 0, 255);
                glColor3ub(c, c, c);
            }
            glVertex2f(p1.x, p1.y);
            glVertex2f(p2.x, p2.y);

            glVertex2f(p2.x, p2.y);
            glVertex2f(p3.x, p3.y);

            glVertex2f(p3.x, p3.y);
            glVertex2f(p1.x, p1.y);
            glEnd();
        }

        if (GraphicSettings::fillTriangles)
        {   glColor3ub(tri.color.x, tri.color.y, tri.color.z);
            glBegin(GL_TRIANGLES);
            glVertex2f(p1.x, p1.y);
            glVertex2f(p2.x, p2.y);
            glVertex2f(p3.x, p3.y);
            glEnd();
        }

        glColor3ub(255, 255, 255);
    }
};

bool LinePlaneIntersect(Vec3& lineStart, Vec3& lineEnd, Triangle& plane, Vec3* pointIntersecting)
{
    // Plane: A*x + B*y + C*z + D = 0 ---> N_x(x-x0) + N_y(y-y0) + N_z(z-z0) = 0
    // Point on plane: x0,y0,z0
    // Point on line: x,y,z
    // Parametric Line: P = P0 + Vt ---> lineEnd = lineStart + (lineEnd-lineStart)t
    // 1st. Paramiterize line like this ---> x = (P0_x + V_x*t), y =, z = ... 
    // 2nd. Plugin x,y,z it into plane equation and solve for t.
    // 3rd. Use t to find the point intersecting both the line and plane.
    Vec3 n = plane.Normal();
    Vec3 pointPlane = plane.verts[0];// plane.Centroid();
    Vec3 v = lineEnd - lineStart;

    double t = (round(DotProduct(n, pointPlane - lineStart))) / DotProduct(n, v);
    Vec3 pIntersect = lineStart + (v * t);
    
    if (round(DotProduct((pIntersect - lineStart), v)) > 0.0)
    {
        *pointIntersecting = pIntersect;
        return true;
    }
    
    return false;
}

bool PointInsideTriangle(const Vec3 &p, const Triangle &tri)
{
    Vec3 A = tri.verts[0];
    Vec3 B = tri.verts[1];
    Vec3 C = tri.verts[2];
    float w1 = ( ((p.x-A.x)*(C.y-A.y)) - ((p.y-A.y)*(C.x-A.x)) ) / ( ((B.x-A.x)*(C.y-A.y)) - ((B.y-A.y)*(C.x-A.x)) );
    float w2 = ( (p.y - A.y) - (w1*(B.y-A.y)) ) / (C.y-A.y);

    return ((w1 >= 0.0 && w2 >= 0.0) && (w1 + w2) <= 1.0);
}

List<Triangle>* triBuffer = new List<Triangle>(10000);

class Transform
{
public:
    Vec3 scale = Vec3(1, 1, 1);
    Vec3 position = Vec3(0, 0, 0);
    Matrix3x3 rotation = Identity3x3;
    Vec3 Forward() { return this->rotation * Vec3D::forward; }
    Vec3 Back() { return this->rotation * Vec3D::back; }
    Vec3 Right() { return this->rotation * Vec3D::right; }
    Vec3 Left() { return this->rotation * Vec3D::left; }
    Vec3 Up() { return this->rotation * Vec3D::up; }
    Vec3 Down() { return this->rotation * Vec3D::down; }
    Vec3 LocalPosition() { return Matrix4x4(this->TranslationMatrix4x4Inverse()) * Vec4(position.x, position.y, position.z, 1); }

    Transform(float scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
    {
        this->scale.x = scale;
        this->scale.y = scale;
        this->scale.z = scale;
        this->position = position;
        this->rotation = YPR(rotationEuler.x, rotationEuler.y, rotationEuler.z);
    }

    Matrix4x4 ScaleMatrix4x4()
    {
        float matrix[4][4] =
{
            {this->scale.x, 0, 0, 0},
            {0, this->scale.y, 0, 0},
            {0, 0, this->scale.z, 0},
            {0, 0, 0, 1}
        };

        return Matrix4x4(matrix);
    }

    Matrix4x4 RotationMatrix4x4() 
    {
        float matrix[4][4] = 
        {
            {this->rotation.m[0][0], this->rotation.m[0][1], this->rotation.m[0][2], 0},
            {this->rotation.m[1][0], this->rotation.m[1][1], this->rotation.m[1][2], 0},
            {this->rotation.m[2][0], this->rotation.m[2][1], this->rotation.m[2][2], 0},
            {0, 0, 0, 1}
        };

        return Matrix4x4(matrix);
    }

    Matrix4x4 TranslationMatrix4x4() 
    {
        float matrix[4][4] = 
        {
            {1, 0, 0, this->position.x},
            {0, 1, 0, this->position.y},
            {0, 0, 1, this->position.z},
            {0, 0, 0, 1}
        };

        return Matrix4x4(matrix);
    }

    Matrix4x4 TranslationMatrix4x4Inverse() 
    {
        float matrix[4][4] = 
        {
            {1, 0, 0, -this->position.x},
            {0, 1, 0, -this->position.y},
            {0, 0, 1, -this->position.z},
            {0, 0, 0, 1}
        };

        return Matrix4x4(matrix);
    }

    Matrix4x4 TRS() 
    {
        return TranslationMatrix4x4() * RotationMatrix4x4() * ScaleMatrix4x4();
    }

    Matrix4x4 TR() 
    {
        return TranslationMatrix4x4() * RotationMatrix4x4();
    }

    Matrix4x4 TRInverse() 
    {
        return Matrix4x4::Transpose(RotationMatrix4x4()) * TranslationMatrix4x4Inverse();
    }
};

class Camera : public Transform
{
public:
    static Camera* main;
    static List<Camera*> cameras;
    static int cameraCount;

    string name;
    Camera(Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
    : Transform(1, position, rotationEuler)
    {
        cameras.emplace(cameras.begin() + cameraCount++, this);
        name = "Camera " + cameraCount;
    }
};
List<Camera*> Camera::cameras = List<Camera*>(1);
int Camera::cameraCount = 0;
Camera* cam = new Camera();
Camera* Camera::main = cam;


class Mesh : public Transform
{
public:
    static List<Mesh*> meshes;
    static int meshCount;
    static int worldTriangleDrawCount;
    
    List<Vec3>* vertices;
    List<Triangle>* triangles;
    virtual List<Triangle>* MapVertsToTriangles() { return triangles; }
    Color color = RGB::white;

    Mesh(float scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
    : Transform(scale, position, rotationEuler)
    {
        Mesh::meshes.emplace(meshes.begin()+meshCount++, this);
        MapVertsToTriangles();
    }

    ~Mesh()
    {
        delete vertices;
        delete triangles;

        for (size_t i = 0; i < meshes.size(); i++)
        {
            if (this == meshes.at(i)) {
                meshes.erase(meshes.begin() + i);
                break;
            }
        }
        meshes.resize(meshCount--);
        meshes.shrink_to_fit();
    }

    static void DrawMeshes()
    {
        static bool init = false;
        if (!init) {
            meshes.resize(meshCount);
            meshes.shrink_to_fit();
            init = true;
        }

        // Transform
        for (int i = 0; i < Mesh::meshCount; i++)
        {
            Mesh::meshes[i]->transformTriangles();
        }

        Mesh::worldTriangleDrawCount = triBuffer->size();

        // Depth Sort (painting algorithm)
        sort(triBuffer->begin(), triBuffer->end(), [](const Triangle& triA, const Triangle& triB) -> bool
            {
                return triA.centroid.w > triB.centroid.w;
            });

        //Draw
        for (int i = 0; i < triBuffer->size(); i++)
        {
            Triangle::DrawTriangle(triBuffer->at(i));
        }

        triBuffer->clear();
    }

private:
    void transformTriangles() 
    {
        // Scale/Distance ratio culling
        bool tooSmallToSee = scale.SqrMagnitude() / (position - Camera::main->position).SqrMagnitude() < 0.00000125;
        if (tooSmallToSee) {
            return;
        }
        
        Matrix4x4 modelToWorldMatrix = this->TRS();
        Matrix4x4 worldToViewMatrix = Camera::main->TRInverse();
        Matrix4x4 projectionMatrix = ProjectionMatrix();
        //Matrix4x4 mvp = projectionMatrix * worldToViewMatrix * modelToWorldMatrix;
        
        //Transform Triangles
        List<Triangle>* tris = MapVertsToTriangles();
        for (int i = 0; i < tris->size(); i++)
        {
            Triangle worldSpaceTri;
            Triangle camSpaceTri;
            Triangle tri = tris->at(i);
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
            };

            //------------------- Normal/Frustum Culling (view space)------------------------
            Vec3 p1_c = camSpaceTri.verts[0];
            Vec3 p2_c = camSpaceTri.verts[1];
            Vec3 p3_c = camSpaceTri.verts[2];
            Vec3 centroid_c = camSpaceTri.Centroid();

            bool tooCloseToCamera = (p1_c.z >= nearClippingPlane || p2_c.z >= nearClippingPlane || p3_c.z >= nearClippingPlane || centroid_c.z >= nearClippingPlane);
            bool tooFarFromCamera = (p1_c.z <= farClippingPlane || p2_c.z <= farClippingPlane || p3_c.z <= farClippingPlane || centroid_c.z <= farClippingPlane);
            bool behindCamera = DotProduct(centroid_c.Normalized(), Vec3D::forward) <= 0.0;
            if (tooCloseToCamera || tooFarFromCamera || behindCamera) {
                continue; // Skip triangle if it's out of cam view.
            }

            // Calculate triangle suface Normal
            Vec3 a = p3_c - p1_c;
            Vec3 b = p2_c - p1_c;
            Vec3 normal_c = (CrossProduct(a, b)).Normalized();

            if (GraphicSettings::invertNormals) {
                normal_c = normal_c * -1.0;
            }

            if (GraphicSettings::culling)
            {
                // Back-face culling - Checks if the triangles backside is facing the camera.
                Vec3 posRelativeToCam = centroid_c;// Since camera is (0,0,0) in view space, the displacement vector from camera to centroid IS the centroid itself.
                bool faceVisibleToCamera = DotProduct(posRelativeToCam.Normalized(), normal_c) <= 0;

                if (!faceVisibleToCamera) {
                    continue;// Skip triangle if it's out of cam view or it's part of the other side of the mesh.
                }
            }

            //------------------------ Lighting (world space)------------------------
            Color triColor = this->color;
            if (GraphicSettings::lighting && GraphicSettings::fillTriangles)
            {
                Vec3 lightSource = Vec3D::up + Vec3D::right + Vec3D::back * .3;
                float amountFacingLight = DotProduct((Vec3)worldSpaceTri.Normal(), lightSource);
                Color colorLit = (triColor * Clamp(amountFacingLight, 0.15, 1));

                triColor = colorLit;
            }
            
            if (GraphicSettings::debugNormals)
            {
                //---------Draw point at centroid and a line from centroid to normal (view space & projected space)-----------
                Vec2 projectedNormal = projectionMatrix * (centroid_c + normal_c);
                Vec2 projectedCentroid = projectionMatrix * centroid_c;
                Point(projectedCentroid.x, projectedCentroid.y);
                Line(projectedCentroid.x, projectedCentroid.y, projectedNormal.x, projectedNormal.y);
            }

            // ================ SCREEN SPACE ==================
            // Project to screen space (image space) 

            //Project single triangle from 3D to 2D
            Triangle projectedTri;
            for (int j = 0; j < 3; j++) {
                Vec4 cameraSpacePoint = camSpaceTri.verts[j];
                projectedTri.verts[j] = projectionMatrix * cameraSpacePoint;
            };
            projectedTri.centroid = projectionMatrix * centroid_c;
            projectedTri.color = triColor;  

            //------------------Ray casting (world & view space)-------------------------------------------------------------
            Vec3 lineStart = Camera::cameras[1]->position;
            Vec3 lineEnd = lineStart + Camera::cameras[1]->Forward() * abs(farClippingPlane);
            Vec3 pointOfIntersection;
            if (LinePlaneIntersect(lineStart, lineEnd, worldSpaceTri, &pointOfIntersection))
            {
                Vec4 pointOfIntersectionProj = projectionMatrix * worldToViewMatrix * pointOfIntersection;
                if (PointInsideTriangle(pointOfIntersectionProj, projectedTri))
                {
                    Vec4 lStartProj = projectionMatrix * worldToViewMatrix * lineStart;
                    Vec4 lEndProj = projectionMatrix * worldToViewMatrix * lineEnd;
                    Line(lStartProj.x, lStartProj.y, lEndProj.x, lEndProj.y); 
                    Point(pointOfIntersectionProj.x, pointOfIntersectionProj.y);
                    projectedTri.color = RGB::white;
                }
            }

            if (GraphicSettings::vfx)
            {
                Vec3 screenLeftSide = Vec3(-1, 0, 0);
                Vec3 screenRightSide = Vec3(1, 0, 0);
                bool leftHalfScreenX = DotProduct(screenLeftSide, (((Vec3)projectedTri.Centroid()) - screenLeftSide).Normalized()) < -0.5;

                if (leftHalfScreenX) {
                    projectedTri.color = Color(0, 0, 255);// std::cout << "Inside" << std::endl;
                }
                else {
                    projectedTri.color = RGB::red;
                }
            }

            //Add projected tri
            triBuffer->push_back(projectedTri);
        }
    }
};
List<Mesh*> Mesh::meshes = List<Mesh*>(1000);
int Mesh::meshCount = 0;
int Mesh::worldTriangleDrawCount = 0;

class CubeMesh : public Mesh
{
public:
    CubeMesh(int scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
        :Mesh(scale, position, rotationEuler) { }

    List<Triangle>* MapVertsToTriangles()
    {
        static int calls = 0;
        if (calls < 1)
        {
            // Local Space (Object Space)
            this->vertices = new List<Vec3>({//new Vec3[8] {
                //south
                Vec3(-1, -1, 1),
                Vec3(-1, 1, 1),
                Vec3(1, 1, 1),
                Vec3(1, -1, 1),
                //north
                Vec3(-1, -1, -1),
                Vec3(-1, 1, -1),
                Vec3(1, 1, -1),
                Vec3(1, -1, -1)
            });

            triangles = new List<Triangle>();
            triangles->reserve(36 / 3);

            //south
            triangles->push_back(Triangle((*vertices)[0], (*vertices)[1], (*vertices)[2]));
            triangles->push_back(Triangle((*vertices)[0], (*vertices)[2], (*vertices)[3]));
            //                                                     
            triangles->push_back(Triangle((*vertices)[7], (*vertices)[6], (*vertices)[5]));
            triangles->push_back(Triangle((*vertices)[7], (*vertices)[5], (*vertices)[4]));

            triangles->push_back(Triangle((*vertices)[3], (*vertices)[2], (*vertices)[6]));
            triangles->push_back(Triangle((*vertices)[3], (*vertices)[6], (*vertices)[7]));

            triangles->push_back(Triangle((*vertices)[4], (*vertices)[5], (*vertices)[1]));
            triangles->push_back(Triangle((*vertices)[4], (*vertices)[1], (*vertices)[0]));

            triangles->push_back(Triangle((*vertices)[1], (*vertices)[5], (*vertices)[6]));
            triangles->push_back(Triangle((*vertices)[1], (*vertices)[6], (*vertices)[2]));

            triangles->push_back(Triangle((*vertices)[3], (*vertices)[7], (*vertices)[4]));
            triangles->push_back(Triangle((*vertices)[3], (*vertices)[4], (*vertices)[0]));
        }

        return triangles;
    }
};

class Plane : Mesh
{
public:
    Plane(int scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
        :Mesh(scale, position, rotationEuler) {}
    
    List<Triangle>* MapVertsToTriangles()
    {
        static int calls = 0;
        if (calls < 1)
        {
            this->vertices = new List<Vec3> {
                Vec3(-1, -1, 0),
                Vec3(-1, 1, 0),
                Vec3(1, 1, 0),
                Vec3(1, -1, 0)
            };

            this->triangles = new List<Triangle>{
                Triangle((*vertices)[0], (*vertices)[1], (*vertices)[2]),
                Triangle((*vertices)[0], (*vertices)[2], (*vertices)[3])
            };
        }

        return triangles;
    }
};

Mesh* LoadMeshFromOBJFile(string objFile) {
    // ----------- Read object file -------------
    List<string> strings;
    string line;
    std::ifstream file;

    file.open(objFile);
    if (file.is_open())
    {
        while (file) {
            // 1st. Gets the next line.
            // 2nd. Seperates each word from that line then stores each word into the strings array.
            getline(file, line);
            string s;
            stringstream ss(line);
            while (getline(ss, s, ' ')) {
                strings.push_back(s);
            }
        }
    }
    file.close();

    // -----------------Construct new mesh-------------------
    // v = vertex
    // f = face.
    List<Vec3>* verts = new List<Vec3>();
    verts->reserve(100);
    List<Triangle>* triangles = new List<Triangle>();
    triangles->reserve(100);

    for (size_t i = 0; i < strings.size(); i++)
    {
        string str = strings[i];

        if (str == "v") {
            float x = stof(strings[++i]);
            float y = stof(strings[++i]);
            float z = stof(strings[++i]);
            verts->push_back(Vec3(x, y, z));
            //std::cout << "(" << x << ", " << y << ", " << z << ")" << endl;
        }
        else if (str == "f") { // means the next 3 strings will be the indices for mapping vertices
            int p3Index = stof(strings[++i]);
            int p2Index = stof(strings[++i]);
            int p1Index = stof(strings[++i]);
            Triangle tri = Triangle(verts->at(p1Index - 1), verts->at(p2Index - 1), verts->at(p3Index - 1));
            triangles->push_back(tri);
        }
    }

    Mesh* mesh = new Mesh();
    mesh->vertices = verts;
    mesh->triangles = triangles;

    return mesh;
}
#endif