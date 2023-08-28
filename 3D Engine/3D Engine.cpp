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

Mesh* textHelloWorld;
Mesh* planet;
Mesh* spaceShip;
CubeMesh* obj1;
CubeMesh* obj2;
CubeMesh* obj3;
CubeMesh* obj4;
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

    GraphicSettings::matrixMode = true;
    //GraphicSettings::debugAxes = true;

    Mesh* cube1 = new CubeMesh(1, Vec3(-5, -5, -10));
    CubeMesh* cube2 = new CubeMesh(1, Vec3(-5, 5, -20));
    CubeMesh* cube3 = new CubeMesh(1, Vec3(5, 5, -30));
    CubeMesh* cube5 = new CubeMesh(1, Vec3(-5, -5, 10));
    CubeMesh* cube6 = new CubeMesh(1, Vec3(-5, 5, 20));
    CubeMesh* cube7 = new CubeMesh(1, Vec3(5, 5, 30));
    CubeMesh* cube8 = new CubeMesh(10, Vec3(5, -5, 40));

    planet = LoadMeshFromOBJFile("Objects/Sphere.obj");
    planet->scale = Vec3(500, 500, 500);
    planet->position += Direction::forward * 1000;
    planet->color = RGB::white;
    
    textHelloWorld = LoadMeshFromOBJFile("Objects/Hello3DWorldText.obj");
    textHelloWorld->scale = Vec3(2, 2, 2);
    textHelloWorld->position = Vec3(0, 0, -490);
    textHelloWorld->color = RGB::green;
    
    spaceShip = LoadMeshFromOBJFile("Objects/SpaceShip_2.2.obj");
    spaceShip->position = Direction::left * 30 + Direction::forward * 10;

    Mesh* parent = new CubeMesh();
    Mesh* child = new CubeMesh();
    Mesh* grandchild = new CubeMesh();
    child->parent = parent;
    grandchild->parent = child;

    obj1 = new CubeMesh(1, Vec3(0, 10, 50), Vec3(0, 90, 0));
    obj2 = new CubeMesh(2, Vec3(0, 0, -2), Vec3(0, 90, 0));
    obj3 = new CubeMesh(2, Vec3(0, 0, -2), Vec3(0, 90, 0));
    obj4 = new CubeMesh(2, Vec3(0, 0, -2), Vec3(0, 90, 0));
    obj2->SetParent(obj1);
    obj3->SetParent(obj2);
    obj4->SetParent(obj3);
    obj1->color = RGB::red;
    obj2->color = RGB::orange;
    obj3->color = RGB::yellow;
    obj4->color = RGB::green;
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
}
 
void Update()
{
   
}

extern List<Point>* pointbuffer;
extern List<Line>* lineBuffer;
void Draw()
{
    Mesh::DrawMeshes();

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
