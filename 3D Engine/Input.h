#pragma once
#ifndef INPUT_H
#define INPUT_H
#include <GLFW/glfw3.h>
#include <Graphics.h>
#include <Physics.h>
#include <functional>
extern CubeMesh* parent;
extern CubeMesh* child;
extern CubeMesh* grandchild;
extern CubeMesh* greatGrandchild;
//-----------------Input----------------------
static double deltaMouseX;
static double deltaMouseY;
float mouseSensitivity = .1;
bool mouseCameraControlEnabled = true;


void OnMouseMoveEvent(GLFWwindow* window, double mouseX, double mouseY)
{
    if (mouseCameraControlEnabled)
    {
        static double screenCenterX = (screenWidth / 2.0);
        static double screenCenterY = (screenHeight / 2.0);

        glfwGetCursorPos(window, &mouseX, &mouseY);

        // Subtract centerX and centerY since the mouse will be forced to move back to the center of the screen before the next frame.
        deltaMouseX = mouseX - screenCenterX;
        deltaMouseY = mouseY - screenCenterY;

        // Reset cursor to the center of the screen
        glfwSetCursorPos(window, screenCenterX, screenCenterY);
        static float rad = PI / 180;
        double xAngle = rad * mouseSensitivity * -deltaMouseY;// *deltaTime;
        double yAngle = numeric_limits<float>::epsilon() + rad * mouseSensitivity * -deltaMouseX;// * deltaTime; // 0.0000001 so no gimbal lock
        
        if (CameraSettings::outsiderViewPerspective) {
            Camera::projector->rotation *= YPR(xAngle, yAngle, 0);
        } else {
            Camera::main->rotation *= YPR(xAngle, yAngle, 0);
        }
        //Camera::main->rotation = Matrix3x3::RotX(rotateSpeed * -deltaMouseY) * Camera::main->rotation * Matrix3x3::RotY((0.00001 + rotateSpeed) * -deltaMouseX);
    }
}
float massFactor = 1;

Transform* grabbing;
Vec3 grabOffset;
Transform* grabbingsOriginalParent = NULL;
std::function<PhysicsObject* ()> spawn = []() { return new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider()); };

void OnMouseButtonEvent(GLFWwindow* window, int button, int action, int mods)
{
    std::cout << "Mouse button:" << button << std::endl;

     auto Throw = [&](PhysicsObject &obj) mutable {
        obj.position = Camera::main->position + (Camera::main->Forward() * 10);
        obj.rotation = Camera::main->rotation;
        obj.velocity = velocity + obj.Forward() * 25;
        obj.mass = massFactor;
        };

    if (action == GLFW_PRESS)
    {
        // Spawn/Throw Mesh
        if (button == 0) {
            PhysicsObject* obj = spawn();// new PhysicsObject(new CubeMesh(), new BoxCollider());//LoadMeshFromOBJFile("Objects/Sphere.obj");
            Throw(*obj);
            obj->mass = massFactor;
            obj->mesh->SetColor(Color::orange);
        }
        // Spawn Mesh
        if (button == 1) {
            PhysicsObject* obj = spawn();// new PhysicsObject(new CubeMesh(), new BoxCollider());//LoadMeshFromOBJFile("Objects/Sphere.obj");
            obj->position = Camera::main->position + (Camera::main->Forward() * 10);
            obj->rotation = Camera::main->rotation;
            obj->isKinematic = true;
            obj->mesh->SetColor(Color::blue);
        }
        else if (button == 2) {
            if (!grabbing)
            {
                static int maxDist = 1000000;
                RaycastInfo<Mesh> info;
                if (Raycast<Mesh>(Camera::main->position, Camera::main->position + Camera::main->Forward() * maxDist, info))
                {
                    cout << "RAYCAST HIT" << '\n';
                    Line::AddWorldLine(Line(Camera::main->position, Camera::main->position + Camera::main->Forward() * maxDist, Color::green, 3));
                    Point::AddWorldPoint(Point(info.contactPoint, Color::green, 10));
                    grabbing = info.objectHit->root;
                    grabbingsOriginalParent = grabbing->parent;
                    grabbing->SetParent(Camera::main);
                }
            }
        }
    }
    if (action == GLFW_RELEASE)
    {
        if (button == 2) {
            if (grabbing) {
                grabbing->SetParent(grabbingsOriginalParent);
                grabbing = NULL;
                grabbingsOriginalParent = NULL;
            }
        }
    }
}

void OnScrollEvent(GLFWwindow* window, double xOffset, double yOffset)
{
    FOV(fieldOfViewDeg - yOffset);
    std::cout << "FOV:" << ToDeg(fov) << "°" << std::endl;
    massFactor += yOffset;
}

void OnKeyPressEvent(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS)
    {
        // Reset Camera
        if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS || key == GLFW_KEY_BACKSPACE) {
            Camera::main->rotation = Matrix3x3::identity;
            Camera::main->position = Vec3();
        }
        // Switch between Cameras
        else if (glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS) {
            Camera::main = Camera::cameras[1];
        }
        else if (glfwGetKey(window, GLFW_KEY_F2) == GLFW_PRESS) {
            Camera::main = Camera::cameras[2];
        }
        else if (glfwGetKey(window, GLFW_KEY_F3) == GLFW_PRESS) {
            CameraSettings::outsiderViewPerspective = !CameraSettings::outsiderViewPerspective;
            Camera::projector->rotation = Matrix3x3::identity;
            Camera::projector->position = Vec3();
        }

        // Spawn
        else if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) 
        {
            spawn = []() { return new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider()); };
        }
        else if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) 
        {
            spawn = []() { return new PhysicsObject(new CubeMesh(), new BoxCollider()); }; //= spawnCube;
        }
        else if (glfwGetKey(window, GLFW_KEY_7) == GLFW_PRESS) {
            Mesh* mesh = LoadMeshFromOBJFile("Diamond.obj");
            mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
            mesh->rotation = Camera::main->rotation;
            mesh->scale *= 0.1;
            mesh->SetColor(Color::red);
        }
        else if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS) {
            Mesh* mesh = LoadMeshFromOBJFile("Icosahedron.obj");
            mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
            mesh->rotation = Camera::main->rotation;
            mesh->scale *= 0.1;
            mesh->SetColor(Color::purple);
        }

        //------------------Physics-------------------

        // Toggle Player's Momentum
        else if (key == GLFW_KEY_X) {
            velocity = Vec3(0, 0, 0);//Reset every toggle state
            isKinematic = !isKinematic;
        }

        // Toggle Player's Inertial Dampeners
        else if (key == GLFW_KEY_Z) {
            dampenersActive = !dampenersActive;
        }

        // Toggle Collision Detection
        else if (key == GLFW_KEY_BACKSLASH) {
            Physics::collisionDetection = !Physics::collisionDetection;
        }
        // Toggle Dynamic Physics
        else if (key == GLFW_KEY_P) {
            Physics::dynamics = !Physics::dynamics;
        }
        // Toggle Gravity
        else if (key == GLFW_KEY_G) {
            Physics::gravity = !Physics::gravity;
        }

        //-------------------Debugging------------------------
        else if (key == GLFW_KEY_I) {
            Graphics::invertNormals = !Graphics::invertNormals;
        }
        else if (key == GLFW_KEY_N) {
            Graphics::debugNormals = !Graphics::debugNormals;
        }
        else if (key == GLFW_KEY_V) {
            Graphics::backFaceCulling = !Graphics::backFaceCulling;
        }
        else if (key == GLFW_KEY_F) {
            Graphics::fillTriangles = !Graphics::fillTriangles;
        }
        else if (key == GLFW_KEY_M) {
            Graphics::displayWireFrames = !Graphics::displayWireFrames;
        }
        else if (key == GLFW_KEY_COMMA) {
            Graphics::debugAxes = !Graphics::debugAxes;
        }
        else if (key == GLFW_KEY_LEFT_BRACKET) {
                Physics::collisionDetection = true;
                Graphics::debugSphereCollisions = !Graphics::debugSphereCollisions;
                Graphics::debugBoxCollisions = !Graphics::debugBoxCollisions;
        }
        else if (key == GLFW_KEY_RIGHT_BRACKET) {
            Physics::collisionDetection = true;
            Graphics::debugPlaneCollisions = !Graphics::debugPlaneCollisions;
        }
        else if (key == GLFW_KEY_L) {
            Graphics::lighting = !Graphics::lighting;
        }
        else if (key == GLFW_KEY_ESCAPE)
        {
            mouseCameraControlEnabled = !mouseCameraControlEnabled;
        }
        else if (key == GLFW_KEY_F4)
        {
            Graphics::vfx = !Graphics::vfx;
        }
        else if (key == GLFW_KEY_F5)
        {
            Graphics::matrixMode = !Graphics::matrixMode;
        }
        else if (key == GLFW_KEY_F6)
        {
            parent->rotation *= Matrix3x3::RotY(ToRad(10));
        }
        else if (key == GLFW_KEY_F7)
        {
            child->rotation *= Matrix3x3::RotY(ToRad(10));
        }
        else if (key == GLFW_KEY_F8)
        {
            grandchild->rotation *= Matrix3x3::RotY(ToRad(10));
        }
        else if (key == GLFW_KEY_F9)
        {
            greatGrandchild->rotation *= Matrix3x3::RotY(ToRad(10));
        }
        // Toggle Transform Hierarchy
        else if (glfwGetKey(window, GLFW_KEY_CAPS_LOCK) == GLFW_PRESS) {
            Transform::parentHierarchyDefault = !Transform::parentHierarchyDefault;
        }
        else if (key == GLFW_KEY_TAB)
        {
            Physics::raycasting = !Physics::raycasting;

            RaycastInfo<Mesh> info;
            if (Raycast<Mesh>(Camera::main->position, Camera::main->position + Camera::main->Forward() * 10, info))
            {
                cout << "RAYCAST HIT" << '\n';
            }
        }
    }
}

extern GLFWwindow* window;
static void CameraControl(Camera* cam)
{
    moveDir = Vec3(0, 0, 0);
    //----------Camera Controls-------
    // FORWARD
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        moveDir += cam->Forward();
    }
    // BACK
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        moveDir += cam->Back();
    }
    // LEFT
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        moveDir += cam->Left();
    }
    // RIGHT
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        moveDir += cam->Right();
    }
    // UP
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        moveDir += cam->Up();
    }
    // DOWN
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
        moveDir += cam->Down();
    }

    moveDir.Normalize();

    // ROTATE CCW
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        cam->rotation *= Matrix3x3::RotZ(rotateSpeed * deltaTime);
    }
    // ROTATE CW
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        cam->rotation *= Matrix3x3::RotZ(-rotateSpeed * deltaTime);
    }
    // LOOK UP
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        cam->rotation *= Matrix3x3::RotX(rotateSpeed * deltaTime);
    }
    // LOOK DOWN
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        cam->rotation *= Matrix3x3::RotX(-rotateSpeed * deltaTime);
    }
    // TURN LEFT
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        cam->rotation *= Matrix3x3::RotY(rotateSpeed * deltaTime);
    }
    // TURN RIGHT
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        cam->rotation *= Matrix3x3::RotY(-rotateSpeed * deltaTime);
    }
    // Speed 
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    {
        accel = defaultAcceleration * 5;
    }
    else {
        accel = defaultAcceleration;
    }
}

static void Input()
{
    if (CameraSettings::outsiderViewPerspective)
    {
        CameraControl(Camera::projector);
    }
    else 
    {
        CameraControl(Camera::main);
    }
}

#endif