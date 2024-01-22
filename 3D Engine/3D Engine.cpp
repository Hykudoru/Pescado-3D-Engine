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
Mesh* planet;
Mesh* spaceShip;
Mesh* spaceShip2;
Mesh* spaceShip3;
CubeMesh* parent;
CubeMesh* child;
CubeMesh* grandchild;
CubeMesh* greatGrandchild;
PhysicsObject* physicsObj;    

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

    planet = LoadMeshFromOBJFile("Sphere.obj");
    planet->scale = Vec3(500, 500, 500);
    planet->position += Direction::forward * 1000;
    planet->color = &RGB::white;
    
    giantText = LoadMeshFromOBJFile("PescadoTextThickLime.obj");// "PescadoText.obj");//"Hello3DWorldText.obj");
    giantText->scale = Vec3(2.5, 2.5, 2.5);
    giantText->position = Vec3(50, 25, -490);
   // textHelloWorld->color = &RGB::green;
    
    spaceShip = LoadMeshFromOBJFile("SpaceShip_2.2.obj");
    spaceShip->position = Direction::left * 30 + Direction::forward * 10;

    spaceShip2 = LoadMeshFromOBJFile("SpaceShip_3.obj");
    spaceShip2->position = Direction::right * 40 + Direction::forward * 100;
    spaceShip2->rotation = Matrix3x3::RotY(PI);

    spaceShip3 = LoadMeshFromOBJFile("SpaceShip_5.obj");
    spaceShip3->position = Direction::right * 20 + Direction::up * 10;

    parent = new CubeMesh(1, Vec3(0, 10, 2), Vec3(0, 45, 0));
    child = new CubeMesh(1, Vec3(0, 0, 2), Vec3(0, 45, 0));
    grandchild = new CubeMesh(2, Vec3(0, 0, 2), Vec3(0, 45, 0));
    greatGrandchild = new CubeMesh(1, Vec3(0, 0, 2), Vec3(0, 45, 0));
    child->SetParent(parent);
    grandchild->SetParent(child);
    greatGrandchild->SetParent(grandchild);
    parent->color = &RGB::red;
    child->color = &RGB::orange;
    grandchild->color = &RGB::yellow;
    greatGrandchild->color = &RGB::green;
    //Mesh* guitar = LoadMeshFromOBJFile("Objects/Guitar.obj");
    //guitar->position += (Camera::main->Forward() * 10) + Camera::main->Right();
    //Mesh* chair = LoadMeshFromOBJFile("Objects/Chair.obj");
    //chair->position += (Camera::main->Forward() * 10) + Camera::main->Left();

    //Plane* plane = new Plane(1, Vec3(0, 0, 0), Vec3(0, 0, 0));
    /*for (size_t i = 1; i < Camera::cameras.size(); i++)//starts at 1 to avoid projector camera
    {
       Mesh* cameraMesh = LoadMeshFromOBJFile("Objects/Camera.obj");
       cameraMesh->SetParent(Camera::cameras[i]);
    }*/

    physicsObj = new PhysicsObject(LoadMeshFromOBJFile("Sphere.obj"), new SphereCollider());
    //physicsObj->collider->isStatic = true;

    for (int i = -10; i < 10; i++)
    {
        for (int j = -10; j < 10; j++)
        {
            PhysicsObject* block = new PhysicsObject(new CubeMesh(), new BoxCollider());
            block->scale *= 2;
            block->collider->isStatic = true;
            block->position = Direction::down*15 + Direction::left*i*10 + Direction::back * j*10;
        }
    }

    Graphics::debugPlaneCollisions = true;
    PhysicsObject* ground = new PhysicsObject(new CubeMesh(0.1), new PlaneCollider(Direction::up));
    ground->collider->isStatic = true;
    ground->position = Direction::down * 15;

    physicsObj = ground;


}
bool temp = false;
void Update()
{
    if (temp) {
        physicsObj->position = Camera::main->position + Camera::main->Forward() * 5;
        physicsObj->rotation = Camera::main->rotation;
    }
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
