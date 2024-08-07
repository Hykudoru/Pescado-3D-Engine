#pragma once
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <Matrix.h>
#include <Graphics.h>
#include <Physics.h>
#include <Input.h>
using namespace std;
GLFWwindow* window;

// Checking if DEBUGGING true in other scripts before using cout also ensures readable slow incremental output.
bool DEBUGGING = false;
void Debug()
{
    //debugging
    DEBUGGING = false;
    static double coutTimer = 0;
    coutTimer += deltaTime;
    if (coutTimer > 1.0 && coutTimer < 1.0 + deltaTime)
    {
        DEBUGGING = true;
        coutTimer = 0;
    }
     
    if (DEBUGGING)
    {
        std::cout << "--------GRAPHICS-------" << endl;
        std::cout << "FPS:" << fps << std::endl;
        std::cout << "Frame Time:" << 1.0 / (double)fps << std::endl;
        std::cout << "Meshes:" << Mesh::count << std::endl;
        std::cout << "Triangles Drawn:" << Mesh::worldTriangleDrawCount << std::endl;
    }
}

Mesh* giantText;
PhysicsObject* planet;
Mesh* sun;
Mesh* moon;
Mesh* spaceShip;
Mesh* spaceShip2;
Mesh* spaceShip3;
CubeMesh* parent;
CubeMesh* child;
CubeMesh* grandchild;
CubeMesh* greatGrandchild; 
Mesh* compass;
//PhysicsObject temp = PhysicsObject(new CubeMesh(), new BoxCollider()); 
Mesh* bender;

PhysicsObject* trigger;

void Init0(GLFWwindow* window)
{
    glfwSetCursorPosCallback(window, OnMouseMoveEvent);
    glfwSetScrollCallback(window, OnScrollEvent);
    glfwSetMouseButtonCallback(window, OnMouseButtonEvent);
    glfwSetKeyCallback(window, OnKeyPressEvent);
    glfwGetWindowSize(window, &screenWidth, &screenHeight);
    glfwSetCursorPos(window, screenWidth / 2.0, screenHeight / 2.0);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

    glLineWidth(2);
    glPointSize(2);
}

void Init(GLFWwindow* window)
{
    glfwSetCursorPosCallback(window, OnMouseMoveEvent);
    glfwSetScrollCallback(window, OnScrollEvent);
    glfwSetMouseButtonCallback(window, OnMouseButtonEvent);
    glfwSetKeyCallback(window, OnKeyPressEvent);
    glfwGetWindowSize(window, &screenWidth, &screenHeight);
    glfwSetCursorPos(window, screenWidth / 2.0, screenHeight / 2.0);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

    glLineWidth(2);
    glPointSize(2);

    Graphics::matrixMode = true;

    Transform* cam2Origin = new Transform(1, Vec3(0, 50, 0), Vec3(ToRad(-90), 0, 0));
    camera2->SetParent(cam2Origin, false);
    camera2->localPosition = Vec3::zero;
    camera2->localRotation = Matrix3x3::identity;

    for (size_t i = 1; i < Camera::cameras.size(); i++)//starts at 1 to avoid projector camera
    {
        Mesh* cameraMesh = LoadMeshFromOBJFile("Camera.obj");
        Camera::cameras[i]->SetMesh(cameraMesh);
    }

    //GraphicSettings::debugAxes = true;/*
    compass = LoadMeshFromOBJFile("Compass.obj");
    compass->localScale *= 0.1;
    compass->ignoreLighting = true;
    compass->forceWireFrame = true;

    sun = LoadMeshFromOBJFile("Sun.obj");
    sun->localRotation = Matrix3x3::RotX(ToRad(45));
    sun->localPosition = lightSource * 100000;
    sun->localScale *= 5000;
    sun->ignoreLighting = true;

    Camera* sunCam = new Camera();
    sunCam->SetParent(sun, false);
    sunCam->localScale = sun->LocalScale4x4Inverse() * sunCam->localScale;
    sunCam->localPosition = Vec3::zero;
    sunCam->localRotation = Matrix3x3::identity;

    planet = new PhysicsObject(500.0, Direction::forward * 1200, Matrix3x3::identity, LoadMeshFromOBJFile("Planet.obj"), new SphereCollider());
    planet->mass = 100000;

    moon = LoadMeshFromOBJFile("Moon-Lowpoly.obj");
    //moon->localPosition += Direction::forward * 500;
    moon->localScale *= 70;
    moon->localPosition = planet->Position() + 1.3*(500*-Direction::forward + 400*Direction::left) + 100*Direction::up;

    giantText = LoadMeshFromOBJFile("Hello3DWorldText.obj");
    giantText->localScale *= 2.5;
    giantText->localPosition = Vec3(0, 25, -490);
    
    spaceShip = LoadMeshFromOBJFile("SpaceShip_2.2.obj");
    spaceShip->localPosition = Direction::left * 30 + Direction::forward * 10;

    spaceShip2 = LoadMeshFromOBJFile("SpaceShip_3.obj");
    spaceShip2->localPosition = Direction::right * 40 + Direction::forward * 100;
    spaceShip2->localRotation = Matrix3x3::RotY(PI);

    spaceShip3 = LoadMeshFromOBJFile("SpaceShip_5.obj");
    spaceShip3->localPosition = Direction::right * 20 + Direction::up * 10;
   
    parent = new CubeMesh(3, Vec3(0, 10, 500), Vec3(0, 45, 0));
    child = new CubeMesh(.5, Vec3(0, 0, 2), Vec3(0, 45, 0));
    grandchild = new CubeMesh(.5, Vec3(0, 0, 2), Vec3(0, 45, 0));
    greatGrandchild = new CubeMesh(.5, Vec3(0, 0, 2), Vec3(0, 45, 0));
    child->SetParent(parent, false);
    grandchild->SetParent(child, false);
    greatGrandchild->SetParent(grandchild, false);
    parent->SetColor(Color::red);
    child->SetColor(Color::orange);
    grandchild->SetColor(Color::yellow);
    greatGrandchild->SetColor(Color::green);
    /*
    PhysicsObject* test = new PhysicsObject(new CubeMesh(), new BoxCollider(true));
    test->localScale = { 1, 2, 1 };
    test->localPosition = Direction::forward * 5;*/

    for (int r = -1; r < 1; r++)
    {
        for (int c = -1; c < 1; c++)
        {
            for (int d = -1; d < 1; d++)
            {
                PhysicsObject* block = new PhysicsObject(new CubeMesh(), new BoxCollider(true));
                block->localScale *= .1;
                block->localPosition = Direction::down * (5+(block->localScale.y*d)) + Direction::left * r * block->localScale.x + Direction::back * c * block->localScale.z;
            }
        }
    }

    /*PhysicsObject* obj = new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new BoxCollider());
   obj->collider->onCollision = [&](Collider* other) mutable {
        if (obj->mesh && other && other->mesh)
        {
            Color c = other->mesh->GetColor();
            other->mesh->SetColor(obj->mesh->GetColor());
            obj->mesh->SetColor(c);
        }
    };*/ 

    //2 X 2 X 2
    /*for (int width = -1; width <= 1; width += 2)
    {
        for (int height = -1; height <= 1; height+=2)
        {
            for (int depth = -1; depth <= 1; depth+=2)
            {
                PhysicsObject* block = new PhysicsObject(new CubeMesh(), new BoxCollider(true));
                block->localScale *= 1;
                //block->SetParent(mesh, false);
                block->localPosition = Direction::right * width * .5 + Direction::up * height * .5 + Direction::forward * depth * .5;

            }
        }
    }*/

    Transform* parent1 = new Transform(2, Direction::up * 50, Vec3(0, ToRad(-90), 0));
    Transform* prev = nullptr;
    for (size_t i = 0; i < 10; i++)
    {
      float scale = .8;
        Transform *t = new CubeMesh(.8, Direction::forward*5, Vec3(0, ToRad(45), 0));
        if (prev == nullptr) {
            prev = t;
            prev->SetParent(parent1, false);
        }
        else {
            t->SetParent(prev, false);
        }
        float s = 2.0 / (i+1 * scale);
        Mesh *c = new CubeMesh(s, t->Position() + t->Right());
        c->SetColor((Color::red * (1.0 / i)));
        t->localPosition += t->localRotation * Direction::right;
        prev = t;
    }
    /*
    Physics::dynamics = false;
    for (size_t i = 0; i < 100; i++)
    {
        PhysicsObject* obj = new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider());
        obj->mass = i*10;
        obj->velocity = Direction::back * 1;
        obj->localPosition = Direction::right * i * 5;
        obj->collider->coefficientRestitution = 0.1;
        PhysicsObject* obj2 = new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider());
        obj2->mass = 100;
        obj2->velocity = Direction::forward * 1;
        obj2->localPosition = obj->Position() + Direction::back * 5;
        obj2->collider->coefficientRestitution = 0.1;
    }*/
    /*
    Physics::dynamics = false;
    for (size_t i = 1; i < 100; i++)
    {
        PhysicsObject* obj = new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider());
        //obj->collider->OnCollision = [&](Collider* other) {cout << typeid(*other->object->mesh).name() << endl; };
        //obj->mass = i;
        obj->localPosition = Direction::forward * i * 5;
        obj->collider->coefficientRestitution = 0.1;
    }
    */
    /*
    PhysicsObject* ground = new PhysicsObject(100, Direction::down * 20, Matrix3x3::identity, new PlaneMesh(), new PlaneCollider(Direction::up, true));
    PhysicsObject* leftWall = new PhysicsObject(100, Direction::left * 50, Matrix3x3::RotZ(ToRad(-90)), new PlaneMesh(), new PlaneCollider(Direction::right, true));
    PhysicsObject* rightWall = new PhysicsObject(100, Direction::right * 50, Matrix3x3::RotZ(ToRad(90)), new PlaneMesh(), new PlaneCollider(Direction::left, true));
    PhysicsObject* backWall = new PhysicsObject(100, Direction::forward * 50, Matrix3x3::RotX(ToRad(90)), new PlaneMesh(), new PlaneCollider(Direction::back, true));
    PhysicsObject* frontWall = new PhysicsObject(100, Direction::back * 50, Matrix3x3::RotX(ToRad(-90)), new PlaneMesh(), new PlaneCollider(Direction::forward, true));
    */
    
    //physicsObj = new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider());
    //physicsObj->scale *= 2;
    //physicsObj->mesh->SetVisibility(false);
    //physicsObj->collider->mesh->SetVisibility(true);
    //physicsObj->collider->isTrigger = true;

    bender = LoadMeshFromOBJFile("Bender.obj");
    bender->localRotation = Matrix3x3::RotZ(ToRad(20));
    bender->localPosition = Camera::main->Position() + (Camera::main->Forward() + Camera::main->Right() * 3);
    /*
    for (size_t i = 0; i < 500; i++)
    {
        auto obj = new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider());
        obj->localPosition += Vec3::one * .1*i;
    }*/
    
    /* //trigger = new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider());
    trigger = new PhysicsObject(new CubeMesh(), new BoxCollider());
    trigger->IsTrigger(true);
    trigger->localScale = Vec3::one * 5;
    trigger->collider->mesh->SetVisibility(true);

    trigger->collider->onCollision = [&](Collider* other) {
        if (other != trigger->collider)
        {
           Vec3 dir = (trigger->Position() - other->Position()).Normalized();
            Vec3 a = dir * 100.0 * deltaTime;
            other->object->velocity += a;
            other->object->mesh->SetColor(Color::gray);

            cout << typeid(*other).name() << endl;
            cout << a.Magnitude() << endl;
        }
    };

    //*/
}

void Update()
{/*
    OctTree<Mesh>::Update();
    Foreach<Mesh>(ManagedObjectPool<Mesh>::objects, [](Mesh* obj) {
        obj->SetColor(Color::red);
    });
    auto list = OctTree<Mesh>::Search(Camera::main->Position(), [](Mesh* obj) { obj->SetColor(Color::green); });
    
    if (DEBUGGING) {
        cout << list->size() << endl;
    }
*/
    /*
    for (int width = -100; width < 100; width++)
    {
        for (int depth = -100; depth < 100; depth++)
        {
            Vec3 from1 = Direction::right* width;
            Vec3 to1 = Direction::right * width + Direction::forward * depth;
            Vec3 from2 = Direction::forward * width;
            Vec3 to2 = Direction::forward * width + Direction::right * depth;
            if (DotProduct(Camera::main->Forward(), from1 - Camera::main->Position()) < 0
             && DotProduct(Camera::main->Forward(), to1 - Camera::main->Position()) < 0
             || DotProduct(Camera::main->Forward(), from2 - Camera::main->Position()) < 0
             && DotProduct(Camera::main->Forward(), from2 - Camera::main->Position()) < 0)
            {
                continue;
            }
            Line::AddWorldLine(Line(from1, to1));
            Line::AddWorldLine(Line(from2, to2));
        }
    }*/

    if (CameraSettings::displayReticle)
    {
        Point::AddPoint(Point(Vec3(), Color::white, 5));
    }

    if (sun)
    {
        if (grabbing != sun)
        {
            float r = sun->localPosition.Magnitude();
            float vSpeed = (((2.0 * PI * r)) / 365.0) * deltaTime;
            Vec3 directionTowardCenter = -sun->localPosition.Normalized();
            Vec3 centripitalAccel = directionTowardCenter * ((vSpeed * vSpeed) / r) * deltaTime;
            Vec3 v = CrossProduct(directionTowardCenter, Direction::up) * vSpeed;
            sun->localPosition += v + centripitalAccel;
        }

        lightSource = sun->Position().Normalized();
    }

    if (compass)
    {
        compass->localPosition = Camera::main->Position() + (Camera::main->Forward() + (Camera::main->Down() * .5) + (Camera::main->Right() * 0.8));
    }

    if (planet) {
        float planetRotationSpeed = ((2 * PI) / 240) * deltaTime;
        planet->localRotation = Matrix3x3::RotX(-planetRotationSpeed) * Matrix3x3::RotY(planetRotationSpeed + 0.000001) * planet->localRotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
    }

    if (spaceShip)
    {
        float shipRotationSpeed = (10 * PI / 180) * deltaTime;
        spaceShip->localRotation = Matrix3x3::RotY(-shipRotationSpeed) * spaceShip->localRotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
        spaceShip->localPosition += spaceShip->Forward() * 20 * deltaTime;
    }
    if (spaceShip2)
    {
        float shipRotationSpeed = (5 * PI / 180) * deltaTime;
        spaceShip2->localRotation = Matrix3x3::RotZ(shipRotationSpeed) * Matrix3x3::RotY(-shipRotationSpeed) * spaceShip2->localRotation;// *spaceShip2->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
        spaceShip2->localPosition += spaceShip2->Forward() * 10 * deltaTime;
    }
    if (spaceShip3)
    {
        float shipRotationSpeed = (20 * PI / 180) * deltaTime;
        spaceShip3->localRotation = Matrix3x3::RotY(shipRotationSpeed) * Matrix3x3::RotZ(shipRotationSpeed) * spaceShip3->localRotation;// *spaceShip2->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
        spaceShip3->localPosition += spaceShip3->Forward() * 15 * deltaTime;

    }

    if (bender)
    {
        bender->localPosition += Direction::left * deltaTime * 0.7;
        bender->localRotation *= Matrix3x3::RotY(5.0 * deltaTime);
    }

    /*
    static float t = 0;
    t += deltaTime;

    for (int i = 0; i < Mesh::objects.size(); i++)
    {
        Mesh* mesh = Mesh::objects[i];
        for (int ii = 0; ii < mesh->vertices.size(); ii++)
        {
            Vec3* v = &((mesh->vertices))[ii];

            //DECAY
            //*v += *v * 0.001*cos(2.0*PI*t*ii);

            //FLUID
            //*v += (*v * (0.001 * sin(2.0 * PI * t + ((float)ii))));

            //WAVE
            //*v += (*v * (0.005 * sin(2.0 * PI * t + ii)));

            //CRUSH
            //*v += (v->Normalized() * (0.001 * sin(2.0 * PI * t + ((float)ii))));
            //*v += (v->Normalized() * (0.001 * sin(t * ii / 2.0) / cos((2.0 * PI / 4.0) * t)));

            //STRETCH
            //*v += (RandomDirection() * ii * (0.001 * sin((2.0 * PI / 4.0) * t)));

            //STRETCH 2
            //*v += (RandomDirection()* ii *(0.001 * abs(sin((2.0 * PI / 4.0) * t ))));

            //Stretch 3
            //*v += (RandomDirection() * -ii * (0.001 * t * abs(sin(t * .1))));
        }
    }
    */
}

int main(void)
{
    //GLFWwindow* window;
    
    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(screenWidth, screenHeight, "Pescado Engine", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    
    //glewInit();
    
    {
        Init(window);
    }

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);
        {
            Time();
            Input();
            Physics();
            Update();
            Draw();
            Debug();
        }

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
