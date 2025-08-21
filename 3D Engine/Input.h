#pragma once
#ifndef INPUT_H
#define INPUT_H
#include <GLFW/glfw3.h>
#include <Graphics.h>
#include <Physics.h>
#include <Utility.h>
#include <functional>
extern CubeMesh* parent;
extern CubeMesh* child;
extern CubeMesh* grandchild;
extern CubeMesh* greatGrandchild;
float massFactor = 1;
Transform* grabbing;
RaycastInfo<Mesh> grabInfo;
Color grabbingsOriginalTriColor; 
Transform* grabbingsOriginalParent = nullptr;
float throwSpeed = 20;
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
            Camera::projector->localRotation *= YPR(xAngle, yAngle, 0);
        } else {
            Camera::main->localRotation *= YPR(xAngle, yAngle, 0);
        }
        //Camera::main->rotation = Matrix3x3::RotX(rotateSpeed * -deltaMouseY) * Camera::main->rotation * Matrix3x3::RotY((0.00001 + rotateSpeed) * -deltaMouseX);
    }
}

std::function<PhysicsObject* ()> spawn = []() { return new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider()); };

void OnMouseButtonEvent(GLFWwindow* window, int button, int action, int mods)
{
    if (mouseCameraControlEnabled)
    {
        auto Throw = [&](PhysicsObject& obj) mutable {
            obj.localPosition = Camera::main->Position() + (Camera::main->Forward() * 8 * obj.mesh->bounds->max.Magnitude() * 0.5);
            obj.localRotation = Camera::main->Rotation();
            obj.velocity = velocity + Camera::main->Forward() * throwSpeed;
            };

        if (action == GLFW_PRESS)
        {
            // Spawn Dynamic
            if (button == 0)
            {
                PhysicsObject* obj = spawn();
                Throw(*obj);
                obj->mass = massFactor;
                obj->collider->coefficientRestitution = 1.0;
            }
            // Raycast and Spawn a kinematic object beside and aligned with object intersecting the ray.
            else if (button == 2)
            {
                PhysicsObject* obj = spawn();// new PhysicsObject(new CubeMesh(), new BoxCollider());//LoadMeshFromOBJFile("Objects/Sphere.obj");
                obj->isKinematic = true;

                static int maxDist = 1000000;
                RaycastInfo<Collider> info;
                if (Raycast(Camera::main->Position(), Camera::main->Position() + Camera::main->Forward() * maxDist, info))
                {
                    info.objectHit->mesh->SetVisibility(true);
                    Vec3 scale = info.objectHit->Scale();
                    Matrix3x3 rot = ExtractRotation(info.objectHit->Parent().TRSInverse() * info.objectHit->LocalRotation4x4());
                    Vec3 pos = info.objectHit->Position() + info.triangleHit_w.Normal() * scale.x;// *2.0;

                    obj->localScale = scale;
                    obj->localRotation = rot;
                    obj->localPosition = pos;
                    obj->mesh->SetColor(Color::yellow);
                }
                else {
                    obj->localPosition = Camera::main->Position() + (Camera::main->Forward() * 10);
                    obj->localRotation = Camera::main->Rotation();
                    obj->localScale *= massFactor;
                    obj->mesh->SetColor(Color::blue);
                }
            }
            else if (button == 1)
            {
                if (!grabbing)
                {
                    static int maxDist = 1000000;
                    if (Raycast<Mesh>(Camera::main->Position(), Camera::main->Position() + Camera::main->Forward() * maxDist, grabInfo))
                    {
                        cout << "RAYCAST HIT" << '\n';
                        Line::AddWorldLine(Line(Camera::main->Position(), Camera::main->Position() + Camera::main->Forward() * maxDist, Color::green, 3));
                        Point::AddWorldPoint(Point(grabInfo.contactPoint, Color::green, 10));
                        grabbing = &grabInfo.objectHit->Root();//grabInfo.objectHit;
                        grabbingsOriginalParent = &grabbing->Parent();
                        grabbingsOriginalTriColor = grabInfo.triangleHit->color;
                        //grabInfo.objectHit->forceWireFrame = true;
                        //grabInfo.triangleHit->forceWireFrame = true;
                        grabInfo.triangleHit->color = Color::green;
                        grabbing->SetParent(Camera::main);
                    }
                }
            }
        }

        if (action == GLFW_RELEASE)
        {
            if (button == 1) {
                if (grabbing) {
                    grabInfo.triangleHit->forceWireFrame = false;
                    grabInfo.triangleHit->color = grabbingsOriginalTriColor;
                    grabbing->SetParent(grabbingsOriginalParent);
                    grabbing = NULL;
                    grabbingsOriginalParent = NULL;
                    grabInfo = RaycastInfo<Mesh>();
                }
            }
        }
    }
}

void OnScrollEvent(GLFWwindow* window, double xOffset, double yOffset)
{
    if (mouseCameraControlEnabled)
    {
        FOV(abs(fieldOfViewDeg - yOffset));
    }
}

void OnKeyPressEvent(GLFWwindow* window, int key, int scancode, int action, int mods)
{

    if (action == GLFW_PRESS)
    {
        static int mainCamIndex = 1;

        // Reset Camera
        if (key == GLFW_KEY_0 || key == GLFW_KEY_BACKSPACE) {
            Camera::main->localRotation = Matrix3x3::identity;
            Camera::main->localPosition = Vec3();
            FOV(60);
        }
        // Switch between Cameras
        else if (key == GLFW_KEY_F2) {
            if (++mainCamIndex >= Camera::cameras.size()) {
                mainCamIndex = 1;
            }
            Camera::main = Camera::cameras[mainCamIndex];
        }
        else if (key == GLFW_KEY_F3) {
            CameraSettings::outsiderViewPerspective = !CameraSettings::outsiderViewPerspective;
            Camera::projector->localRotation = Matrix3x3::identity;
            Camera::projector->localPosition = Vec3();
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
        else if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
        {
            spawn = []() { 
                auto obj = new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider());
                obj->collider->isStatic = true;
                obj->mesh->SetColor(Color::white);
                return obj;
                };
        }
        else if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS)
        {
            spawn = []() { 
                auto obj = new PhysicsObject(new CubeMesh(), new BoxCollider()); 
                obj->collider->isStatic = true;
                obj->mesh->SetColor(Color::white);
                return obj;
                }; //= spawnCube;
        }
        else if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS)
        {
            spawn = []() {
                auto obj = new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider());
                obj->isKinematic = true;
                obj->mesh->SetColor(Color::blue);
                return obj;
                };
        }
        else if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS)
        {
            spawn = []() {
                auto obj = new PhysicsObject(new CubeMesh(), new BoxCollider());
                obj->isKinematic = true;
                obj->mesh->SetColor(Color::blue);
                return obj;
                }; //= spawnCube;
        }
        else if (glfwGetKey(window, GLFW_KEY_7) == GLFW_PRESS) {
            spawn = []() {
                auto obj = new PhysicsObject(LoadMeshFromOBJFile("Diamond.obj"), new BoxCollider(false));
                obj->localScale *= 0.1;
                return obj;
                };
        }
        else if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS) {
            Mesh* mesh = LoadMeshFromOBJFile("Icosahedron.obj");
            mesh->localPosition = Camera::main->Position() + (Camera::main->Forward() * 10);
            mesh->localRotation = Camera::main->Rotation();
            mesh->localScale *= 0.1;
            mesh->SetColor(Color::purple);
        }
        else if (glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS)
        {
            auto obj = spawn();
            obj->localPosition = Camera::main->Position() + (Camera::main->Forward() * 10);
            obj->localRotation = Camera::main->Rotation();
            //obj->localScale *= massFactor;

            if (obj->collider->isStatic)
            {
                obj->mesh->SetColor(Color::white);
            }
            else if(obj->isKinematic) {
                obj->mesh->SetColor(Color::blue);
            }
            else {
                obj->mesh->SetColor(Color::orange + Vec3::one * -massFactor);
            }
        }
        else if (key == GLFW_KEY_DELETE) {
            RaycastInfo<Collider> info;
            if (Raycast(Camera::main->Position(), Camera::main->Position() + Camera::main->Forward() * 100000, info))
            {
                delete info.objectHit->object;
            }
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
        
        if (key == GLFW_KEY_ESCAPE)
        {
            mouseCameraControlEnabled = !mouseCameraControlEnabled;
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
        else if (key == GLFW_KEY_F4)
        {
            Graphics::vfx = !Graphics::vfx;
        }
        else if (key == GLFW_KEY_F5)
        {
            Graphics::matrixMode = !Graphics::matrixMode;
        }
        else if (key == GLFW_KEY_R) {
            Physics::raycastDebugging = !Physics::raycastDebugging;
        }
        else if (key == GLFW_KEY_LEFT_ALT)
        {
            massFactor = Clamp(floor(massFactor) - 10, .1, 1000);
        }
        else if (key == GLFW_KEY_RIGHT_ALT)
        {
            massFactor = Clamp(floor(massFactor) + 10, .1, 1000);
        }
        else if (glfwGetKey(window, GLFW_KEY_CAPS_LOCK) == GLFW_PRESS) {
            Physics::octTree = !Physics::octTree;
        }
        else if (key == GLFW_KEY_TAB)
        {
            Physics::raycasting = !Physics::raycasting;
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
        moveDir += cam->localRotation * Direction::forward;
    }
    // BACK
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        moveDir += cam->localRotation* Direction::back;
    }
    // LEFT
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        moveDir += cam->localRotation * Direction::left;
    }
    // RIGHT
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        moveDir += cam->localRotation * Direction::right;
    }
    // UP
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        moveDir += cam->localRotation * Direction::up;
    }
    // DOWN
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
        moveDir += cam->localRotation* Direction::down;
    }

    moveDir.Normalize();

    // ROTATE CCW
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        cam->localRotation *= Matrix3x3::RotZ(rotateSpeed * deltaTime);
    }
    // ROTATE CW
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        cam->localRotation *= Matrix3x3::RotZ(-rotateSpeed * deltaTime);
    }
    // LOOK UP
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        cam->localRotation *= Matrix3x3::RotX(rotateSpeed * deltaTime);
    }
    // LOOK DOWN
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        cam->localRotation *= Matrix3x3::RotX(-rotateSpeed * deltaTime);
    }
    // TURN LEFT
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        cam->localRotation *= Matrix3x3::RotY(rotateSpeed * deltaTime);
    }
    // TURN RIGHT
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        cam->localRotation *= Matrix3x3::RotY(-rotateSpeed * deltaTime);
    }
    // Speed 
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    {
        accel = defaultAcceleration * 7;
    }
    else if (glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS)
    {
        accel = defaultAcceleration * 3000;
    }
    else {
        accel = defaultAcceleration;
    }
    if (glfwGetKey(window, GLFW_KEY_PAGE_UP) == GLFW_PRESS)
    {
        deltaTime *= 10;
    }
    if (glfwGetKey(window, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS)
    {
        deltaTime *= -10;
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