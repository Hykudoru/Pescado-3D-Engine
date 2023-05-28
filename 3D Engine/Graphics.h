
#include <Matrix.h>
#include <math.h>
#include <vector>

#include <string.h>
const float PI = 3.14159265359f;
#define List std::vector

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
    static bool perspective;
    static bool fillTriangles;
};
bool GraphicSettings::culling = true;
bool GraphicSettings::invertNormals = false;
bool GraphicSettings::debugNormals = false;
bool GraphicSettings::perspective = true;
bool GraphicSettings::fillTriangles = false;

struct World
{
private:
    static float worldScale;
public:
    static Vec3 forward;
    static Vec3 back;
    static Vec3 right;
    static Vec3 left;
    static Vec3 up;
    static Vec3 down;
    
    static void SetScale(float val)
    {
        if (val <= 0.0)
        {
            return;
        }
        worldScale = val;
    }

    static float GetScale()
    {
        return worldScale;
    }
};
float World::worldScale = 1;
Vec3 World::forward = Vec3(0, 0, -1);
Vec3 World::back = Vec3(0, 0, 1);
Vec3 World::right = Vec3(1, 0, 0);
Vec3 World::left = Vec3(-1, 0, 0);
Vec3 World::up = Vec3(0, 1, 0);
Vec3 World::down = Vec3(0, -1, 0);

float aspectRatio = screenWidth / screenHeight;
float nearClippingPlane = -0.1;
float farClippingPlane = -1000.0;
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
}/*
void Points(Vec2 verts[])
{
    glBegin(GL_POINTS);
    glVertex2f(-0.5, -0.5);
    glVertex2f(-0.5, 0.5);

    glVertex2f(0.5, 0.5);
    glVertex2f(0.5, -0.5);
    glEnd();
}*/
void Line(float x0, float y0, float x, float y)
{
    glBegin(GL_LINES);
    glVertex2f(x0, y0);
    glVertex2f(x, y);
    glEnd();
}

Vec2 ProjectPoint(Vec4 point)
{
    if (GraphicSettings::perspective) {
        point = perspectiveProjectionMatrix * point;
    } 
    else {
        point = orthographicProjectionMatrix * point;
    }
    
    point.x *= World::GetScale();
    point.y *= World::GetScale();
    return point;
}

struct Triangle
{
    static int count;
    Vec3 verts[3];
    //Vec3 centroid;
    Triangle() { count++; };
    Triangle(Vec3 p1, Vec3 p2, Vec3 p3)
    {
        this->verts[0] = p1;
        this->verts[1] = p2;
        this->verts[2] = p3;
        count++;
        //centroid...
    }
};
int Triangle::count = 0;

void DrawTriangle(Triangle tri)
{
    Vec4 p1 = tri.verts[0];
    Vec4 p2 = tri.verts[1];
    Vec4 p3 = tri.verts[2];
    
    Point(p1.x, p1.y);
    Point(p2.x, p2.y);

    Point(p2.x, p2.y);
    Point(p3.x, p3.y);

    Point(p3.x, p3.y);
    Point(p1.x, p1.y);

    glBegin(GL_LINES);
    
        glVertex2f(p1.x, p1.y);
        glVertex2f(p2.x, p2.y);

        glVertex2f(p2.x, p2.y);
        glVertex2f(p3.x, p3.y);

        glVertex2f(p3.x, p3.y);
        glVertex2f(p1.x, p1.y);

    glEnd();

    if (GraphicSettings::fillTriangles)
    {
        glBegin(GL_TRIANGLES);

        glVertex2f(p1.x, p1.y);
        glVertex2f(p2.x, p2.y);
        glVertex2f(p3.x, p3.y);

        glEnd();
    }
}


class Transform
{
public:
    //static List<Transform*> transforms;// = [];
    //static int transformCount;

    Vec3 scale = Vec3(1, 1, 1);
    Vec3 position = Vec3(0, 0, 0);
    Matrix3x3 rotation = Identity3x3;
    Vec3 Forward() { return this->rotation * World::forward; }
    Vec3 Back() { return this->rotation * World::back; }
    Vec3 Right() { return this->rotation * World::right; }
    Vec3 Left() { return this->rotation * World::left; }
    Vec3 Up() { return this->rotation * World::up; }
    Vec3 Down() { return this->rotation * World::down; }
    Vec3 LocalPosition() { return Matrix4x4(this->TranslationMatrix4x4Inverse()) * Vec4(position.x, position.y, position.z, 1); }

    Transform(float scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
    {
        this->scale.x = scale;
        this->scale.y = scale;
        this->scale.z = scale;
        this->position = position;
        this->rotation = YPR(rotationEuler.x, rotationEuler.y, rotationEuler.z);
        //Transform::transforms[Transform::transformCount++] = this;
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
//Transform** Transform::transforms = new Transform*[100];
//List<Transform*> Transform::transforms = List<Transform*>(1000);
//int Transform::transformCount = 0;

class Camera : public Transform
{
public:
    static Camera* main;
    static List<Camera*> cameras;
    static int cameraCount;
    //string name;
    Camera(float scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
    : Transform(scale, position, rotationEuler)
    {
        if (Camera::cameraCount == 0) {
            Camera::main = this;
            Camera::cameras = List<Camera*>({this});
        }
        Camera::cameras[Camera::cameraCount++] = this;
        //this.name = "Camera " + Camera.#cameraCount;
    }
};
Camera* cam = new Camera();
Camera* Camera::main;
List<Camera*> Camera::cameras;
int Camera::cameraCount = 0;

//                                      List<Triangle>* triangleBuffer = new List<Triangle>(100);

class Mesh : public Transform
{
public:
    static List<Mesh*> meshes;
    static int meshCount;
    static int worldTriangleDrawCount;
    
    List<Vec3>* vertices;
    List<Triangle>* triangles;
    static List<Triangle> projectedTriangles;
    virtual List<Triangle>* MapVertsToTriangles() { return NULL; }

    Mesh(float scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
    : Transform(scale, position, rotationEuler)
    {
        Mesh::meshes[Mesh::meshCount++] = this;
        //Mesh::worldTriangleCount += (*this->triangles(this->vertices)).length; 
        MapVertsToTriangles();
    }

    static void DrawMeshes() {
        static bool init = false;
        if (!init) { 
            meshes.resize(meshCount);
            init = true;
        }

        Mesh::worldTriangleDrawCount = 0;
        for (int i = 0; i < Mesh::meshCount; i++)
        {
            Mesh::meshes[i]->drawMesh();
        }
    }

    void transformTriangles() 
    {
        //Transform Triangles
        List<Triangle>* tris = MapVertsToTriangles();
        projectedTriangles.resize(tris->size());
        for (int i = 0; i < tris->size(); i++)
        {
            Triangle camSpaceTri;
            Triangle tri = tris->at(i);
            for (int j = 0; j < 3; j++)
            {
                Vec4 vert = tri.verts[j];
                // Homogeneous coords (x, y, z, w=1)
                //vert = new Vec4(vert.x, vert.y, vert.z, 1);

                // =================== WORLD SPACE ===================
                // Transform local coords to world-space coords.

                Matrix4x4 modelToWorldMatrix = TRS();
                Vec4 worldPoint = modelToWorldMatrix * vert;

                // ================ VIEW/CAM/EYE SPACE ================
                // Transform world coordinates to view coordinates.

                Matrix4x4 worldToViewMatrix = Camera::main->TRInverse();
                Vec4 cameraSpacePoint = worldToViewMatrix * worldPoint;
                camSpaceTri.verts[j] = cameraSpacePoint;
            };

            // Still in View/Cam/Eye space
            //-------------------Normal/Culling------------------------
            Vec3 p1 = camSpaceTri.verts[0];
            Vec3 p2 = camSpaceTri.verts[1];
            Vec3 p3 = camSpaceTri.verts[2];
            Vec3 centroid = Vec3((p1.x + p2.x + p3.x) / 3.0, (p1.y + p2.y + p3.y) / 3.0, (p1.z + p2.z + p3.z) / 3.0);

            bool tooCloseToCamera = (p1.z >= nearClippingPlane || p2.z >= nearClippingPlane || p3.z >= nearClippingPlane || centroid.z >= nearClippingPlane);
            bool tooFarFromCamera = (p1.z <= farClippingPlane || p2.z <= farClippingPlane || p3.z <= farClippingPlane || centroid.z <= farClippingPlane);
            bool behindCamera = DotProduct(centroid.Normalized(), World::forward) <= 0;
            if (tooCloseToCamera || tooFarFromCamera || behindCamera) {
                continue; // Skip triangle if it's out of cam view.
            }

            // Calculate triangle suface Normal
            Vec3 a = p3 - p1;
            Vec3 b = p2 - p1;
            Vec3 normal = (CrossProduct(a, b)).Normalized();

            if (GraphicSettings::invertNormals) {
                normal = normal * -1.0;
            }

            if (GraphicSettings::culling)
            {
                // Back-face culling - Checks if the triangles backside is facing the camera.
                Vec3 normalizedFromCamPos = centroid.Normalized();// Since camera is (0,0,0) in view space, the displacement vector from camera to centroid IS the centroid itself.
                bool camVisible = DotProduct(normalizedFromCamPos, normal) <= 0;

                if (!camVisible) {
                    continue;// Skip triangle if it's out of cam view or it's part of the other side of the mesh.
                }
            }

            // ================ SCREEN SPACE ==================
            // Project to screen space (image space) 

            if (GraphicSettings::debugNormals)
            {
                //---------Draw point at centroid and a line from centroid to normal-----------
                Vec2 projectedNormal = ProjectPoint(centroid + normal);
                Vec2 projectedCentroid = ProjectPoint(centroid);
                Point(projectedCentroid.x, projectedCentroid.y);
                Line(projectedCentroid.x, projectedCentroid.y, projectedNormal.x, projectedNormal.y);
            }

            //Project single triangle from 3D to 2D
            Triangle projectedTri;
            for (int j = 0; j < 3; j++) {
                Vec4 cameraSpacePoint = camSpaceTri.verts[j];
                projectedTri.verts[j] = ProjectPoint(cameraSpacePoint);
            };
            //Add projected tri
            projectedTriangles[i] = projectedTri;
        }
    }

    void drawMesh() {
        //strokeWeight(2);
        this->transformTriangles();
        Mesh::worldTriangleDrawCount += this->projectedTriangles.size();
        for (int i = 0; i < projectedTriangles.size(); i++)
        {
            DrawTriangle(projectedTriangles.at(i));
        }
        //projectedTriangles.clear();
    }
};
List<Mesh*> Mesh::meshes = List<Mesh*>(100);
List<Triangle> Mesh::projectedTriangles = List<Triangle>(100);
int Mesh::meshCount = 0;
int Mesh::worldTriangleDrawCount = 0;

class CubeMesh : public Mesh
{
public:
    CubeMesh(int scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
        :Mesh(scale, position, rotationEuler)
    {
        
    }

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
        }
        else {
            triangles->clear();
        }
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

        return triangles;
    }
};