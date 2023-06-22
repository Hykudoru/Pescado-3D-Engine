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
        std::cout << "Triangles Drawn:" << Mesh::worldTriangleDrawCount << std::endl;
    }
}
Mesh* textHelloWorld;
Mesh* planet;
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

    Physics::collisionDetection = false;

    Mesh* cube1 = new CubeMesh(1, Vec3(-5, -5, -10));
    CubeMesh* cube2 = new CubeMesh(1, Vec3(-5, 5, -20));
    CubeMesh* cube3 = new CubeMesh(1, Vec3(5, 5, -30));
    CubeMesh* cube5 = new CubeMesh(1, Vec3(-5, -5, 10));
    CubeMesh* cube6 = new CubeMesh(1, Vec3(-5, 5, 20));
    CubeMesh* cube7 = new CubeMesh(1, Vec3(5, 5, 30));
    CubeMesh* cube8 = new CubeMesh(10, Vec3(5, -5, 40));

    cube1->color = Color(255, 0, 0);
    cube2->color = Color(255, 255, 0);
    cube3->color = Color(0, 255, 255);
    cube5->color = Color(0, 0, 255);

    
    planet = LoadMeshFromOBJFile("Objects/Sphere.obj");
    planet->scale = Vec3(500, 500, 500);
    planet->position += Vec3D::forward * 1000;
    planet->color = RGB::white;
    
    textHelloWorld = LoadMeshFromOBJFile("Objects/Hello3DWorldText.obj");
    textHelloWorld->scale = Vec3(2, 2, 2);
    textHelloWorld->position = Vec3(0, 0, -490);
    textHelloWorld->color = RGB::green;
    
    //Mesh* guitar = LoadMeshFromOBJFile("Objects/Guitar.obj");
    //guitar->position += (Camera::main->Forward() * 10) + Camera::main->Right();
    //Mesh* chair = LoadMeshFromOBJFile("Objects/Chair.obj");
    //chair->position += (Camera::main->Forward() * 10) + Camera::main->Left();

    //Plane* plane = new Plane(1, Vec3(0, 0, 0), Vec3(0, 0, 0));
    Camera* camera2 = new Camera(Vec3(0, 50, 0), Vec3(-90 * PI / 180, 0, 0)); 
}
 
void Update()
{
}
void Draw()
{
    Mesh::DrawMeshes();
}

int main(void)
{
    GLFWwindow* window;
    
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
    Init(window);
    
   
    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);


        Time();
        Input(window);
        Physics(window);
        Update();
        Draw();
        Debug();


        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
