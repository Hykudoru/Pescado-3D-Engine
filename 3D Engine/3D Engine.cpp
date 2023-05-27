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
float deccel = 0.95;
bool isKinematic = false;
bool dampenersActive = true;

Vec3 moveDir = Vec3(0,0,0);
Vec3 velocity = Vec3(0, 0, 0);
//Camera camera2 = Camera(1, new Vec3(0, 50, 0), new Vec3(-90 * Math.PI / 180, 0, 0));

static void Physics(GLFWwindow* window)
{
    //---------- Physics Update ------------
    if (!isKinematic) {
        velocity += moveDir * accel;
        if (dampenersActive) {
            velocity *= 0.95;
        }
        Camera::main->position += velocity;
        //std::cout << "Velocity:" << velocity.ToString();
    }
    else {
        Camera::main->position += moveDir * moveSpeed;
    }

    Mesh::meshes[0]->rotation = Matrix3x3::RotZ(angle) * Mesh::meshes[0]->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
}

//-----------------Input----------------------

static double prevMouseX;
static double prevMouseY;
static double deltaMouseX;
static double deltaMouseY;
static double mouseX;
static double mouseY;
static float rotateSpeed = 0.008;

static void CheckMouseMove(GLFWwindow* window) 
{
    static  double centerX = (screenWidth / 2.0);
    static  double centerY = (screenHeight / 2.0);
    glfwGetCursorPos(window, &mouseX, &mouseY);
    deltaMouseX = mouseX - centerX;
    deltaMouseY = mouseY - centerY;
    glfwSetCursorPos(window, centerX, centerY);

    Camera::main->rotation *= YPR(rotateSpeed * -deltaMouseY, (0.00001 + rotateSpeed)* -deltaMouseX, 0);
    //Camera::main->rotation = Matrix3x3::RotX(rotateSpeed * -deltaMouseY) * Camera::main->rotation * Matrix3x3::RotY((0.00001 + rotateSpeed) * -deltaMouseX);

};

void OnScrollEvent(GLFWwindow* window, double xOffset, double yOffset)
{
    FOV(fieldOfViewDeg - yOffset);

    std::cout << "FOV:" << ToDeg(fov) << "°" << std::endl;
    std::cout << "World Scale:" << World::GetScale() << std::endl;
    std::cout << "Camera Pos:" << Camera::main->position.z << std::endl;
}
GLFWscrollfun onScroll = OnScrollEvent;


static void Input(GLFWwindow* window)
{
    CheckMouseMove(window);
    moveDir = Vec3(0, 0, 0);
    //----------Camera Controls-------
    // FORWARD
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        moveDir += Camera::main->Forward();
    }
    // BACK
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        moveDir += Camera::main->Back();
    }
    // LEFT
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        moveDir += Camera::main->Left();
    }
    // RIGHT
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        moveDir += Camera::main->Right();
    }
    // UP
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        moveDir += Camera::main->Up();
    }
    // DOWN
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
        moveDir += Camera::main->Down();
    }

    moveDir.Normalize();

    // ROTATE CCW
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        Camera::main->rotation *= Matrix3x3::RotZ(PI / 90);
    }
    // ROTATE CW
    else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        Camera::main->rotation *= Matrix3x3::RotZ(-PI / 90);
    }
    // Reset Camera
    else if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS)
    {
        // Camera::main->reset();
        //Fix later
        Camera::main->rotation = Identity3x3;
        Camera::main->position = Vec3();
    }
    else if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) {
        Camera::main = Camera::cameras[0];
    }
    else if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) {
        Camera::main = Camera::cameras[1];
    }


    if (glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS) {
        GraphicSettings::invertNormals = !GraphicSettings::invertNormals;
    }
    if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS) {
        GraphicSettings::debugNormals = !GraphicSettings::debugNormals;
    }
    if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS) {
        GraphicSettings::culling = !GraphicSettings::culling;
    }
    if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
        GraphicSettings::perspective = !GraphicSettings::perspective;
    }

    //------------------Physics-------------------
    // Toggle momentum
    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) {
        velocity = Vec3(0, 0, 0);//Reset every toggle state
        isKinematic = !isKinematic;
    }

    // Toggle dampeners
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
        dampenersActive = !dampenersActive;
    }

    // ----------------Other------------------

    // Spawn Mesh
    if (glfwGetKey(window, GLFW_KEY_APOSTROPHE) == GLFW_PRESS) {
        Mesh* c = new CubeMesh(1);
        c->position = Camera::main->position + (Camera::main->Forward() * 10);
    }

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

CubeMesh cube1(1, Vec3(-5, -5, -10));
CubeMesh cube2(1, Vec3(-5, 5, -20));
CubeMesh cube3(1, Vec3(5, 5, -30));
CubeMesh cube4(10, Vec3(5, -5, -40));
CubeMesh cube5(1, Vec3(-5, -5, 10));
CubeMesh cube6(1, Vec3(-5, 5, 20));
CubeMesh cube7(1, Vec3(5, 5, 30));
CubeMesh cube8(10, Vec3(5, -5, 40));
CubeMesh cube9(100, Vec3(0, 0, -400));
void Init(GLFWwindow* window)
{
    World::SetScale(1);
    Camera::main->position.z = 10;
    glfwSetScrollCallback(window, onScroll);
    glfwGetWindowSize(window, &screenWidth, &screenHeight);
    glfwSetCursorPos(window, screenWidth / 2.0, screenHeight / 2.0);
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
        

        Input(window);
        Physics(window);
        Mesh::DrawMeshes();
 


        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
