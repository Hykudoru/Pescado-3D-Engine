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
#include <iostream>
#include <math.h>
#include <Matrix.h>
#include <Graphics.h>
using namespace std;

double angle = (PI / 4) * 0.005;

float moveSpeed = 1;
float accel = 0.1;
Vec3 velocity = Vec3(0, 0, 0);
bool isKinematic = false;
bool dampenersActive = true;
//Camera* cam1 = new Camera();
//Camera camera2 = Camera(1, new Vec3(0, 50, 0), new Vec3(-90 * Math.PI / 180, 0, 0));

void OnScroll(GLFWwindow* window, double xOffset, double yOffset) 
{
    FOV(fieldOfViewDeg - yOffset);
    
    std::cout << "FOV:" << ToDeg(fov) << "°" << endl;
    std::cout << "World Scale:" << World::GetScale() << endl;
    std::cout << "Camera Pos:" << Camera::main->position.z << endl;
}
GLFWscrollfun onScroll = OnScroll;

void Init(GLFWwindow* window)
{   
    World::SetScale(1);
    Camera::main->position.z = 10;
    glfwSetScrollCallback(window, onScroll);

    CubeMesh* cube1 = new CubeMesh(1, Vec3(-5, -5, -10));
    CubeMesh* cube2 = new CubeMesh(1, Vec3(-5, 5, -20));
    CubeMesh* cube3 = new CubeMesh(1, Vec3(5, 5, -30));
    CubeMesh* cube4 = new CubeMesh(10, Vec3(5, -5, -40));
    CubeMesh* cube5 = new CubeMesh(1, Vec3(-5, -5, 10));
    CubeMesh* cube6 = new CubeMesh(1, Vec3(-5, 5, 20));
    CubeMesh* cube7 = new CubeMesh(1, Vec3(5, 5, 30));
    CubeMesh* cube8 = new CubeMesh(10, Vec3(5, -5, 40));
    CubeMesh* cube9 = new CubeMesh(100, Vec3(0, 0, -400));
}

void Draw(GLFWwindow* window)
{
    //glfwGetWindowSize(window, &screenWidth, &screenHeight);
    //---------- PHYSICS Update ------------
    /*if (!isKinematic) {
        velocity = VectorSum(velocity, VectorScale(moveDir, accel));
        if (dampenersActive) {
            velocity = VectorScale(velocity, 0.95);
        }
        Camera.main.position = VectorSum(Camera.main.position, velocity);
        console.log("Velocity:" + velocity.ToString());
    }
    else {
        Camera.main.position = VectorSum(Camera.main.position, VectorScale(moveDir, moveSpeed));
    }*/
    Mesh::meshes[0]->rotation = Matrix3x3::RotZ(angle) * Mesh::meshes[0]->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
    //Mesh::meshes[2]->rotation *= Matrix3x3::RotZ(angle);// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
    //cube2.rotation = MatrixMultiply(YPR(angle * ((screenWidth / 2) - mouseY), angle * -((screenWidth / 2) - mouseX), 0), cube2.rotation);
    //Camera::main->rotation *= Matrix3x3::RotY(angle);
    


    //----------- DRAW ------------
    Mesh::DrawMeshes();
    /*
    // ------------------------------------
    if (DEBUGGING) {
        let trisIgnored = Mesh.worldTriangleCount - Mesh.worldTriangleDrawCount;
        let trisIncluded = Mesh.worldTriangleCount - trisIgnored;
        let trisPercIncluded = Math.round((trisIncluded / Mesh.worldTriangleCount) * 100);
        let trisPercIgnored = Math.round((trisIgnored / Mesh.worldTriangleCount) * 100);

        std::cout << "Meshes: " << Mesh::meshes.size() << endl;
        //std::cout << "Triangles: " + Mesh::worldTriangleCount << endl;
        std::cout << "Triangles Drawing: " <<  trisIncluded + " ( " + trisPercIncluded + "% )" << endl;
        std::cout << "Triangles Ignored: " <<  trisIgnored + " ( " + trisPercIgnored + "% )</div>");
        $('#debug').append("</br>");
        $('#debug').append("<div>Camera Position: " + Camera.main.position.ToString() + "</div>");
        $('#debug').append("<div>Velocity: " + velocity.ToString() + "</div>");
        $('#debug').append("<div>Speed: " + Math.round(Magnitude(velocity)) + "</div>");
    }*/
}

int main(void)
{
    GLFWwindow* window;
    
    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(800, 800, "3D Graphics Engine", NULL, NULL);
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
        glLineWidth(2);
        glPointSize(2);
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);
        
        Draw(window);

        

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}