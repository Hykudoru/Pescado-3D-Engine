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
    GraphicSettings::debugAxes = true;

    Mesh* cube1 = new CubeMesh(1, Vec3(-5, -5, -10));
    CubeMesh* cube2 = new CubeMesh(1, Vec3(-5, 5, -20));
    CubeMesh* cube3 = new CubeMesh(1, Vec3(5, 5, -30));
    CubeMesh* cube5 = new CubeMesh(1, Vec3(-5, -5, 10));
    CubeMesh* cube6 = new CubeMesh(1, Vec3(-5, 5, 20));
    CubeMesh* cube7 = new CubeMesh(1, Vec3(5, 5, 30));
    CubeMesh* cube8 = new CubeMesh(10, Vec3(5, -5, 40));

    //cube1->color = Color(255, 0, 0);
    //cube2->color = Color(255, 255, 0);
    //cube3->color = Color(0, 255, 255);
    //cube5->color = Color(0, 0, 255);

    
    planet = LoadMeshFromOBJFile("Objects/Sphere.obj");
    planet->scale = Vec3(500, 500, 500);
    planet->position += Vec3D::forward * 1000;
    planet->color = RGB::white;
    
    textHelloWorld = LoadMeshFromOBJFile("Objects/Hello3DWorldText.obj");
    textHelloWorld->scale = Vec3(2, 2, 2);
    textHelloWorld->position = Vec3(0, 0, -490);
    textHelloWorld->color = RGB::green;
    
    Mesh* parent = new CubeMesh();
    Mesh* child = new CubeMesh();
    Mesh* grandchild = new CubeMesh();
    child->parent = parent;
    grandchild->parent = child;

    obj1 = new CubeMesh(1, Vec3(0, 0, 0));
    obj2 = new CubeMesh(1, Vec3(0, 2, -1));
    obj3 = new CubeMesh(1, Vec3(0, 2, -1));
    obj4 = new CubeMesh(1, Vec3(0, 2, -1));

    obj2->rotation = Matrix3x3::RotZ(ToRad(-45));
    obj3->rotation = Matrix3x3::RotZ(ToRad(-45));
    //obj4->rotation = Matrix3x3::RotZ(ToRad(-45));
    obj2->parent = obj1;
    obj3->parent = obj2;
    obj4->parent = obj3;

    //Mesh* guitar = LoadMeshFromOBJFile("Objects/Guitar.obj");
    //guitar->position += (Camera::main->Forward() * 10) + Camera::main->Right();
    //Mesh* chair = LoadMeshFromOBJFile("Objects/Chair.obj");
    //chair->position += (Camera::main->Forward() * 10) + Camera::main->Left();

    //Plane* plane = new Plane(1, Vec3(0, 0, 0), Vec3(0, 0, 0));
    Camera* camera2 = new Camera(Vec3(0, 50, 0), Vec3(-90 * PI / 180, 0, 0)); 
    Mesh* cameraMesh = new CubeMesh();
    cameraMesh->parent = camera2;
}
 
void Update(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_CAPS_LOCK) == GLFW_PRESS) {
        if (obj3->parent) {
            obj3->parent = NULL;
        }
        else {
            obj3->parent = obj2;
        }
    }
   
}

extern List<Point>* points;
extern List<Line>* lines;
void Draw()
{
    Mesh::DrawMeshes();

    for (size_t i = 0; i < lines->size(); i++)
    {
        (*lines)[i].Draw();
    }
    lines->clear();

    for (size_t i = 0; i < points->size(); i++)
    {
        (*points)[i].Draw();
    }

    points->clear();
}
GLFWwindow* window;
int main(void)
{
    //GLFWwindow* window;
    
    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(800, 800, "Pescado Engine", NULL, NULL);
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
            Physics(window);
            Update(window);
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
