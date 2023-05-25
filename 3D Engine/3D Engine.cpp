// 3D Engine.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
/*
#include <iostream>

int main()
{
    std::cout << "Hello World!\n";
}
*/
//#include <GL/glew.h>
#include <GLFW/glfw3.h>
//#include <Z:/3D Engine/3D Engine/Matrix.cpp>

#include <iostream>
#include <math.h>
#include <Matrix.h>
#include <Graphics.h>
using namespace std;


CubeMesh Cube(1, Vec3(-5, -5, -10));
Camera camera;
Matrix3x3 rotation;
void Update()
{
    rotation *= YPR(3.14159/180.0, 10 * 3.14159 / 180.0, 100 * 3.14159 / 180.0);
    std::cout << *rotation.m[0] << endl;
    std::cout << *rotation.m[1] << endl;
    std::cout << *rotation.m[2] << endl;
    //----------- DRAW ------------
    Mesh::DrawMeshes();
}
int main(void)
{
    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(640, 480, "3D Graphics Engine", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    
    //glewInit();

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        glLineWidth(2);
        glPointSize(2);
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);

        Update();

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}