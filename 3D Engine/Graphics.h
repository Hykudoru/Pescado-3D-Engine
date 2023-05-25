
#include <Matrix.h>
#include <math.h>
#include <vector>
#include <string.h>
#define PI 3.14159

float ToDeg(float rad) {
    return rad * 180.0 / PI;
}

float ToRad(float deg) {
    return deg * PI / 180.0;
}
int screenWidth = 700;//700;//screen.width - 20;
int screenHeight = 700; //screen.height - 20; //screen.height;// - 30;
float worldScale = .5;
//-----------GRAPHICS---------------

struct GraphicSettings
{
    static bool culling;
    static bool invertNormals;
    static bool debugNormals;
    static bool perspective;
};
bool GraphicSettings::culling = true;
bool GraphicSettings::invertNormals = false;
bool GraphicSettings::debugNormals = false;
bool GraphicSettings::perspective = true;

struct World
{
    static Vec3 forward;
    static Vec3 back;
    static Vec3 right;
    static Vec3 left;
    static Vec3 up;
    static Vec3 down;
};
Vec3 World::forward(0, 0, -1);
Vec3 World::back = Vec3(0, 0, 1);
Vec3 World::right = Vec3(1, 0, 0);
Vec3 World::left = Vec3(-1, 0, 0);
Vec3 World::up = Vec3(0, 1, 0);
Vec3 World::down = Vec3(0, -1, 0);

float orthographicProjectionMatrix[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 1}
};

float nearClippingPlane = -.1;
float farClippingPlane = -1000;
float fov = 60 * PI / 180.0;

float perspectiveProjectionMatrix[4][4] {
    {1/tan(fov/2), 0, 0, 0},
    {0, 1/tan(fov/2), 0, 0},
    {0, 0, 1, 0},
    {0, 0, -1, 0}
};

/*
void FOV(int deg)
{
    fov = ToRad(deg);
    perspectiveProjectionMatrix[4][4] = {
        {1/tan(fov/2), 0, 0, 0},
        {0, 1/tan(fov/2), 0, 0},
        {0, 0, 1, 0},
        {0, 0, -1, 0}
    };
}*/

Vec4 ProjectPoint(Vec4 point)
{
    if (GraphicSettings::perspective) {
        point = perspectiveProjectionMatrix * point;
    } 
    else {
        point = orthographicProjectionMatrix * point;
    }

    point.x *= worldScale * screenWidth;
    point.y *= worldScale * screenHeight;
    
    return point;
}

void DrawTriangle(Vec4 tri[3]) {
    Vec4 p1 = tri[0];
    Vec4 p2 = tri[1];
    Vec4 p3 = tri[2];

    //Draw Triangle
    //line(p1.x, p1.y, 0, p2.x, p2.y, 0);
    //line(p2.x, p2.y, 0, p3.x, p3.y, 0);
    //line(p3.x, p3.y, 0, p1.x, p1.y, 0);
}

class Triangle
{
public:
    Vec3 verts[3];
    //Vec3 centroid;

    Triangle(Vec3 p1, Vec3 p2, Vec3 p3)
    {
        this->verts[0] = p1;
        this->verts[1] = p2;
        this->verts[2] = p3;

        //centroid...
    }
};

class Transform
{
public:
    static Transform** transforms;// = [];
    static int transformCount;

    Vec3 scale = Vec3(1, 1, 1);
    Vec3 position = Vec3(0, 0, 0);
    Matrix3x3 rotation = Identity3x3;
    Vec3 Forward() { return this->rotation * World::up; }
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
        Transform::transforms[Transform::transformCount++] = this;
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
    }

    Matrix4x4 TranslationMatrix4x4() 
    {
        float matrix[4][4] = {
            {1, 0, 0, this->position.x},
            {0, 1, 0, this->position.y},
            {0, 0, 1, this->position.z},
            {0, 0, 0, 1}
        };

        return Matrix4x4(matrix);
    }

    Matrix4x4 TranslationMatrix4x4Inverse() 
    {
        float matrix[4][4] = {
            {1, 0, 0, -this->position.x},
            {0, 1, 0, -this->position.y},
            {0, 0, 1, -this->position.z},
            {0, 0, 0, 1}
        };

        return Matrix4x4(matrix);
    }

    Matrix4x4 TRS() 
    {
        return this->TranslationMatrix4x4() * this->RotationMatrix4x4() * this->ScaleMatrix4x4();
    }

    Matrix4x4 TR() 
    {
        return this->TranslationMatrix4x4() * this->RotationMatrix4x4();
    }

    Matrix4x4 TRInverse() 
    {
        return Matrix4x4::Transpose(this->RotationMatrix4x4().m) * this->TranslationMatrix4x4Inverse();
    }
};
Transform** Transform::transforms = new Transform*[100];

class Camera : Transform
{
public:
    static Camera* main;
    static Camera** cameras;
    static int cameraCount;
    //string name;

    Camera(float scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
        : Transform(scale, position, rotationEuler)
    {
        if (Camera::cameraCount == 0) {
            Camera::main = this;
        }
        Camera::cameras[Camera::cameraCount++] = this;
        //this.name = "Camera " + Camera.#cameraCount;
    }
};
Camera** Camera::cameras = new Camera*[3];
Camera* Camera::main = &Camera();

class Mesh : Transform
{
public:
    static Mesh** meshes;
    static int meshCount;
    //static int worldTriangleCount;
    static int worldTriangleDrawCount;
    Vec3* vertices; // = []
    Vec3* projectedTriangles; // = []

    Mesh(float scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
        : Transform(scale, position, rotationEuler)
    {
        Mesh::meshes[Mesh::meshCount++] = this;
        //Mesh::worldTriangleCount += (*this->triangles(this->vertices)).length;
    }
protected:
    virtual Vec3* triangles(Vec3* verts) = 0;

    void transformTriangles() 
    {
        //Transform Triangles
        Vec3* tris = this->triangles(this->vertices);
        for (let i = 0; i < tris.length; i++)
        {
            let camSpaceTri = [];
            let tri = tris[i];
            for (let j = 0; j < tri.length; j++)
            {
                let vert = tri[j];// Local 3D (x,y,z)
                // Homogeneous coords (x, y, z, w=1)
                vert = new Vec4(vert.x, vert.y, vert.z, 1);

                // =================== WORLD SPACE ===================
                // Transform local coords to world-space coords.

                let modelToWorldMatrix = this.TRS;
                let worldPoint = Matrix4x4VectorMult(modelToWorldMatrix, vert);

                // ================ VIEW/CAM/EYE SPACE ================
                // Transform world coordinates to view coordinates.

                let worldToViewMatrix = Camera.main.TRInverse;
                let cameraSpacePoint = Matrix4x4VectorMult(worldToViewMatrix, worldPoint);
                camSpaceTri[j] = cameraSpacePoint;
            };

            // Still in View/Cam/Eye space
            //-------------------Normal/Culling------------------------
            let p1 = camSpaceTri[0];
            let p2 = camSpaceTri[1];
            let p3 = camSpaceTri[2];
            let centroid = new Vec3((p1.x + p2.x + p3.x) / 3.0, (p1.y + p2.y + p3.y) / 3.0, (p1.z + p2.z + p3.z) / 3.0);

            let tooCloseToCamera = (p1.z >= nearClippingPlane || p2.z >= nearClippingPlane || p3.z >= nearClippingPlane || centroid.z >= nearClippingPlane);
            let tooFarFromCamera = (p1.z <= farClippingPlane || p2.z <= farClippingPlane || p3.z <= farClippingPlane || centroid.z <= farClippingPlane);
            let behindCamera = DotProduct(Normalized(centroid), forward) <= 0;
            if (tooCloseToCamera || tooFarFromCamera || behindCamera) {
                continue; // Skip triangle if it's out of cam view.
            }

            // Calculate triangle suface Normal
            let a = VectorSub(p3, p1);
            let b = VectorSub(p2, p1);
            let normal = Normalized(CrossProduct(a, b));

            if (GraphicSettings.invertNormals) {
                normal = VectorScale(normal, -1.0)
            }

            if (GraphicSettings.culling)
            {
                // Back-face culling - Checks if the triangles backside is facing the camera.
                let normalizedFromCamPos = Normalized(centroid);// Since camera is (0,0,0) in view space, the displacement vector from camera to centroid IS the centroid itself.
                let camVisible = DotProduct(normalizedFromCamPos, normal) <= 0;

                if (!camVisible) {
                    continue;// Skip triangle if it's out of cam view or it's part of the other side of the mesh.
                }
            }

            // ================ SCREEN SPACE ==================
            // Project to screen space (image space) 

            if (GraphicSettings.debugNormals)
            {
                //---------Draw triangle centroid and normal-----------
                let projCentroidToNormal = ProjectPoint(Vec4.ToVec4(VectorSum(centroid, normal)));
                let projCentroid = ProjectPoint(Vec4.ToVec4(centroid));
                point(projCentroid.x, projCentroid.y, 0);
                line(projCentroid.x, projCentroid.y, 0, projCentroidToNormal.x, projCentroidToNormal.y, 0);
            }

            //Project single triangle from 3D to 2D
            let projectedTri = []
                for (let j = 0; j < 3; j++) {
                    const cameraSpacePoint = camSpaceTri[j];
                    projectedTri[j] = ProjectPoint(cameraSpacePoint);
                };
            //Add projected tri
            projectedTriangles[i] = projectedTri;
        }

        return projectedTriangles;
    }

    drawMesh() {
        strokeWeight(2);
        let projectedTriangles = this.transformTriangles();
        Mesh.worldTriangleDrawCount += projectedTriangles.length;
        projectedTriangles.forEach(tri = > {
            DrawTriangle(tri);
        });
    }

    static DrawMeshes() {
        Mesh.worldTriangleDrawCount = 0;
        Mesh.meshes.forEach(mesh = > {
            mesh.drawMesh();
        });
    }
}

class CubeMesh : Mesh
{
    CubeMesh(int scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
        :Mesh(scale, position, rotationEuler)
    {
        // Local Space (Object Space)
        this->vertices = {
            //south
            Vec3(-1, -1, 1),
            Vec3(-1, 1, 1),
            { x: 1, y : 1, z : 1 },
            { x: 1, y : -1, z : 1 },
            //north
            { x: -1, y : -1, z : -1 },
            { x: -1, y : 1, z : -1 },
            { x: 1, y : 1, z : -1 },
            { x: 1, y : -1, z : -1 },
        };
    }

    Triangle** triangles = NULL;
    void createTriangles(Vec3* verts, int count)
    {
        if (triangles != NULL) {
            return;
        }

        int count = count / 3;
        triangles = new Triangle*[count] {
            //south
            new Triangle(verts[0], verts[1], verts[2]),
            new Triangle(verts[0], verts[2], verts[3]),
            //north
            new Triangle(verts[7], verts[6], verts[5]),
            new Triangle(verts[7], verts[5], verts[4]),
            //right
            new Triangle(verts[3], verts[2], verts[6]),
            new Triangle(verts[3], verts[6], verts[7]),
            //left
            new Triangle(verts[4], verts[5], verts[1]),
            new Triangle(verts[4], verts[1], verts[0]),
            //top
            new Triangle(verts[1], verts[5], verts[6]),
            new Triangle(verts[1], verts[6], verts[2]),
            //bottom
            new Triangle(verts[3], verts[7], verts[4]),
            new Triangle(verts[3], verts[4], verts[0])
        };
    }
};