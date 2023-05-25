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
#include <Matrix.h>
#include <iostream>
#include <math.h>
using namespace std;
const float PI = 3.14159265359f;
Matrix3x3 rotation;
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
        rotation = Matrix3x3::RotZ((PI / 2.0));
        Vec2 v2;
        Vec3 v3;
        Vec4 v4;

        v4 = Vec3(10, 20, 30);
        v3 = Vec4(10, 20, 30, -30);
        std::cout << "(" << v3.x << ", " << v3.y << ", " << v3.z << ")\n";
        //std::cout << "(" << v4.x << ", " << v4.y << ", " << v4.z << ", " << v4.w << ")\n";
        //v3 = v4;
       // glfwGetCursorPos(window, mouseX, mouseY);
       // std::cout << "MouseX: " << mouseX << "\n";
       // std::cout << "MouseY: " << mouseY;
       // InputUpdate();

        glLineWidth(2);
        glPointSize(2);
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);

        glBegin(GL_POINTS);
            glVertex2f(-0.5, -0.5);
            glVertex2f(-0.5, 0.5);

            glVertex2f(-0.5, 0.5);
            glVertex2f(0.5, 0.5);
        glEnd();

        glBegin(GL_LINES);
            glVertex2f(-0.5, -0.5);
            glVertex2f(-0.5, 0.5);

            glVertex2f(-0.5, 0.5);
            glVertex2f(0.5, 0.5);

            glVertex2f(0.5, 0.5);
            glVertex2f(-0.5, -0.5);
        glEnd();
        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}