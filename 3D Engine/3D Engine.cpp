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
    if (coutTimer > 0.25 && coutTimer < 0.25 + deltaTime)
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
Mesh* temp;
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
    //GraphicSettings::debugAxes = true;
    compass = LoadMeshFromOBJFile("Compass.obj");
    compass->scale *= 0.1;
    sun = LoadMeshFromOBJFile("Sun.obj");
    sun->rotation = Matrix3x3::RotX(ToRad(45));
    sun->position = lightSource * 100000;
    sun->scale *= 5000;
    sun->ignoreLighting = true;
    Camera* sunCam = new Camera();
    sunCam->SetParent(sun);
    sunCam->position = Vec3::zero;
    sunCam->rotation = Matrix3x3::identity;
    
    Transform* camOrigin = new Transform(1, Vec3(0, 0, 0), Vec3(ToRad(-90), 0, 0));
    camera2->SetParent(camOrigin);
    camera2->rotation = Matrix3x3::identity;

    planet = new PhysicsObject(500.0, Direction::forward * 1200, Matrix3x3::identity, LoadMeshFromOBJFile("Planet.obj"), new SphereCollider());
    planet->mass = 100000;

    moon = LoadMeshFromOBJFile("Moon.obj");
    //moon->position += Direction::forward * 500;
    moon->scale *= 70;
    moon->position = planet->Position() + 1.3*(500*-Direction::forward + 400*Direction::left) + 100*Direction::up;

    giantText = LoadMeshFromOBJFile("Hello3DWorldText.obj");
    giantText->scale *= 2.5;
    giantText->position = Vec3(0, 25, -490);// Vec3(50, 25, -490);
   // textHelloWorld->color = &Color::green;
    
    spaceShip = LoadMeshFromOBJFile("SpaceShip_2.2.obj");
    spaceShip->position = Direction::left * 30 + Direction::forward * 10;

    spaceShip2 = LoadMeshFromOBJFile("SpaceShip_3.obj");
    spaceShip2->position = Direction::right * 40 + Direction::forward * 100;
    spaceShip2->rotation = Matrix3x3::RotY(PI);

    spaceShip3 = LoadMeshFromOBJFile("SpaceShip_5.obj");
    spaceShip3->position = Direction::right * 20 + Direction::up * 10;

    Mesh* bender = LoadMeshFromOBJFile("Bender.obj");
    bender->position += Direction::back * 3;

    Mesh* computer = LoadMeshFromOBJFile("Computer-Lab-Module.obj");
    computer->position += Direction::back * 5;

    parent = new CubeMesh(3, Vec3(0, 10, 500), Vec3(0, 45, 0));
    child = new CubeMesh(3, Vec3(0, 0, 2), Vec3(0, 45, 0));
    grandchild = new CubeMesh(3, Vec3(0, 0, 2), Vec3(0, 45, 0));
    greatGrandchild = new CubeMesh(3, Vec3(0, 0, 2), Vec3(0, 45, 0));
    child->SetParent(parent, false);
    grandchild->SetParent(child, false);
    greatGrandchild->SetParent(grandchild, false);
    parent->SetColor(Color::red);
    child->SetColor(Color::orange);
    grandchild->SetColor(Color::yellow);
    greatGrandchild->SetColor(Color::green);

    temp = LoadMeshFromOBJFile("Sphere.obj");
    //Mesh* guitar = LoadMeshFromOBJFile("Objects/Guitar.obj");
    //guitar->position += (Camera::main->Forward() * 10) + Camera::main->Right();
    //Mesh* chair = LoadMeshFromOBJFile("Objects/Chair.obj");
    //chair->position += (Camera::main->Forward() * 10) + Camera::main->Left();

    //Plane* plane = new Plane(1, Vec3(0, 0, 0), Vec3(0, 0, 0));

    for (size_t i = 1; i < Camera::cameras.size(); i++)//starts at 1 to avoid projector camera
    {
       Mesh* cameraMesh = LoadMeshFromOBJFile("Camera.obj");
       cameraMesh->SetParent(Camera::cameras[i], false);
       cameraMesh->position += -Direction::forward;
    }

    //physicsObj->collider->isStatic = true;
    
    for (int i = -10; i < 10; i++)
    {
        for (int j = -10; j < 10; j++)
        {
            PhysicsObject* block = new PhysicsObject(new CubeMesh(), new BoxCollider(true));
            block->scale *= 2;
            block->position = Direction::down*15 + Direction::left*i*10 + Direction::back * j*10;
        }
    }
    /*
    PhysicsObject* ground = new PhysicsObject(400, Direction::down * 200, Matrix3x3::identity, new PlaneMesh(), new PlaneCollider(Direction::up, true));
    PhysicsObject* leftWall = new PhysicsObject(300, Direction::left * 200, Matrix3x3::RotZ(ToRad(-90)), new PlaneMesh(), new PlaneCollider(Direction::up, true));
    PhysicsObject* rightWall = new PhysicsObject(300, Direction::right * 200, Matrix3x3::RotZ(ToRad(90)), new PlaneMesh(), new PlaneCollider(Direction::up, true));
    PhysicsObject* backWall = new PhysicsObject(300, Direction::forward * 200, Matrix3x3::RotX(ToRad(90)), new PlaneMesh(), new PlaneCollider(Direction::up, true));
    PhysicsObject* frontWall = new PhysicsObject(new PlaneMesh(300, Direction::back * 200, Vec3(ToRad(-90), 0, 0)), new PlaneCollider(Direction::up, true));
    */
    
    //physicsObj = new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider());
    //physicsObj->scale *= 2;
    //physicsObj->mesh->SetVisibility(false);
    //physicsObj->collider->mesh->SetVisibility(true);
    //physicsObj->collider->isTrigger = true;

}

void Update()
{
    compass->position = Camera::main->Position() + (Camera::main->Forward() + (Camera::main->Down()*.5)+(Camera::main->Right()*0.8));
    temp->position = greatGrandchild->Position() + greatGrandchild->Right();
    if (CameraSettings::displayReticle)
    {
        Point::AddPoint(Point(Vec3(), Color::white, 5));
    }

    if (grabbing != sun)
    {
        float r = sun->position.Magnitude();
        float vSpeed = (((2.0 * PI * r)) / 365.0) * deltaTime;
        Vec3 directionTowardCenter = -sun->position.Normalized();
        Vec3 centripitalAccel = directionTowardCenter * ((vSpeed * vSpeed) / r) * deltaTime;
        Vec3 v = CrossProduct(directionTowardCenter, Direction::up) * vSpeed;
        sun->position += v + centripitalAccel;
        //static float t = 0;
        //t += deltaTime;
        //sun->position = Vec3(r * cos((2.0 * PI)*t/36), 0, r * sin(((2.0 * PI)*t/36)));
    }

    lightSource = sun->Position().Normalized();

    if (planet) {
        float planetRotationSpeed = ((2 * PI) / 240) * deltaTime;
        planet->rotation = Matrix3x3::RotX(-planetRotationSpeed) * Matrix3x3::RotY(planetRotationSpeed + 0.000001) * planet->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
    }

    if (spaceShip)
    {
        float shipRotationSpeed = (10 * PI / 180) * deltaTime;
        spaceShip->rotation = Matrix3x3::RotY(-shipRotationSpeed) * spaceShip->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
        spaceShip->position += spaceShip->Forward() * 20 * deltaTime;
    }
    if (spaceShip2)
    {
        float shipRotationSpeed = (5 * PI / 180) * deltaTime;
        spaceShip2->rotation = Matrix3x3::RotZ(shipRotationSpeed) * Matrix3x3::RotY(-shipRotationSpeed) * spaceShip2->rotation;// *spaceShip2->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
        spaceShip2->position += spaceShip2->Forward() * 10 * deltaTime;
    }
    if (spaceShip3)
    {
        float shipRotationSpeed = (20 * PI / 180) * deltaTime;
        spaceShip3->rotation = Matrix3x3::RotY(shipRotationSpeed) * Matrix3x3::RotZ(shipRotationSpeed) * spaceShip3->rotation;// *spaceShip2->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
        spaceShip3->position += spaceShip3->Forward() * 15 * deltaTime;

    }
    /*
    static float t = 0;
    t += deltaTime;
    
    for (int i = 0; i < Mesh::objects.size(); i++)
    {
        Mesh* mesh = Mesh::objects[i];
        for (int ii = 0; ii < mesh->vertices->size(); ii++)
        {
            Vec3* v = &(*(mesh->vertices))[ii]; 
            
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
    }*/
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
