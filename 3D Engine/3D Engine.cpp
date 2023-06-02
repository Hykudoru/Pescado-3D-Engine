//#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <Matrix.h>
#include <Graphics.h>
using namespace std;

Mesh* LoadMeshFromOBJFile(string objFile)
{
    // ----------- Read object file -------------
    List<string> strings;
    string line;
    std::ifstream file;

    file.open(objFile);
    if (file.is_open())
    {
        while (file) {
            // 1st. Gets the next line.
            // 2nd. Seperates each word from that line then stores each word into the strings array.
            getline(file, line);
            string s;
            stringstream ss(line);
            while (getline(ss, s, ' ')) {
                strings.push_back(s);
            }
        }
    }
    file.close();
        
    // -----------------Construct new mesh-------------------
    // v = vertex
    // f = face.
    List<Vec3>* verts = new List<Vec3>();
    verts->reserve(100);
    List<Triangle>* triangles = new List<Triangle>();
    triangles->reserve(100);
        
    for (size_t i = 0; i < strings.size(); i++)
    {
        string str = strings[i];
            
        if (str == "v") {
            float x = stof(strings[++i]);
            float y = stof(strings[++i]);
            float z = stof(strings[++i]);
            verts->push_back(Vec3(x, y, z));
            //std::cout << "(" << x << ", " << y << ", " << z << ")" << endl;
        }
        else if (str == "f") { // means the next 3 strings will be the indices for mapping vertices
            int p3Index = stof(strings[++i]);
            int p2Index = stof(strings[++i]);
            int p1Index = stof(strings[++i]);
            Triangle tri = Triangle(verts->at(p1Index-1), verts->at(p2Index-1), verts->at(p3Index-1));
            triangles->push_back(tri);
        }
    }

    Mesh* mesh = new Mesh();
    mesh->vertices = verts;
    mesh->triangles = triangles;

    return mesh;
}

float moveSpeed = 50;
float rotateSpeed = PI/2;
float accel = 0.1;
float deccel = 0.95;
bool isKinematic = false;
bool dampenersActive = true;

Vec3 moveDir = Vec3(0, 0, 0);
Vec3 velocity = Vec3(0, 0, 0);

//Camera camera2 = Camera(1, new Vec3(0, 50, 0), new Vec3(-90 * Math.PI / 180, 0, 0));

double deltaTime = 0;
void Time()
{ 
    static double prevTime = 0;
    double currentTime = glfwGetTime();
    deltaTime = currentTime - prevTime;
    prevTime = currentTime;

    //std::cout << deltaTime << std::endl;
}

static void Physics(GLFWwindow* window)
{
    //---------- Physics Update ------------
    if (!isKinematic) {
        velocity += moveDir * moveSpeed * accel;
        if (dampenersActive) {
            velocity *= 0.95;
        }
        Camera::main->position += velocity * deltaTime;
        //std::cout << "Velocity:" << velocity.ToString();
    }
    else {
        Camera::main->position += moveDir * moveSpeed * deltaTime;
    }

    Mesh::meshes[0]->rotation = Matrix3x3::RotZ(2*PI * deltaTime) * Mesh::meshes[0]->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
}

//-----------------Input----------------------

static double prevMouseX;
static double prevMouseY;
static double deltaMouseX;
static double deltaMouseY;
static double mouseX;
static double mouseY;
float mouseSensitivity = .1;
bool mouseCameraControlEnabled = true;

static void CheckMouseMove(GLFWwindow* window) 
{
    if (mouseCameraControlEnabled)
    {
        static  double centerX = (screenWidth / 2.0);
        static  double centerY = (screenHeight / 2.0);

        // Read mouse displacement.
        // Also subtract centerX and centerY since the mouse will move back to center before next frame.
        glfwGetCursorPos(window, &mouseX, &mouseY);
        deltaMouseX = mouseX - centerX;
        deltaMouseY = mouseY - centerY;
        // Reset cursor to center of screen
        glfwSetCursorPos(window, centerX, centerY);

        double xAngle = mouseSensitivity * deltaTime * -deltaMouseY;
        double yAngle = 0.00001 + mouseSensitivity * deltaTime * -deltaMouseX;
        Camera::main->rotation *= YPR(xAngle, yAngle, 0);
        //Camera::main->rotation = Matrix3x3::RotX(rotateSpeed * -deltaMouseY) * Camera::main->rotation * Matrix3x3::RotY((0.00001 + rotateSpeed) * -deltaMouseX);
    }
};

void OnScrollEvent(GLFWwindow* window, double xOffset, double yOffset)
{
    FOV(fieldOfViewDeg - yOffset);

    std::cout << "FOV:" << ToDeg(fov) << "°" << std::endl;
}

void OnMouseButtonEvent(GLFWwindow* window, int button, int action, int mods)
{
    std::cout << "Mouse button:" << button << std::endl;
}


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
        Camera::main->rotation *= Matrix3x3::RotZ(rotateSpeed * deltaTime);
    }
    // ROTATE CW
    else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        Camera::main->rotation *= Matrix3x3::RotZ(-rotateSpeed * deltaTime);
    }
    // Reset Camera
    else if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS) {
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

    //------------------Physics-------------------
    // 
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
        Mesh* mesh = LoadMeshFromOBJFile("Objects/Sphere.obj");
        mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
    }

    //-------------------Debugging------------------------

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
    if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS) {
        GraphicSettings::fillTriangles = !GraphicSettings::fillTriangles;
        GraphicSettings::displayWireFrames = !GraphicSettings::displayWireFrames;
    }
    
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        mouseCameraControlEnabled = !mouseCameraControlEnabled;
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

GLFWscrollfun onScroll = OnScrollEvent;
GLFWmousebuttonfun onMouseButton = OnMouseButtonEvent;
void Init(GLFWwindow* window)
{
    glfwSetScrollCallback(window, onScroll);
    glfwSetMouseButtonCallback(window, onMouseButton);
    glfwGetWindowSize(window, &screenWidth, &screenHeight);
    glfwSetCursorPos(window, screenWidth / 2.0, screenHeight / 2.0);

    World::SetScale(1);
    Camera::main->position.z = 10;
    CubeMesh* cube1 = new CubeMesh(1, Vec3(-5, -5, -10));
    CubeMesh* cube2 = new CubeMesh(1, Vec3(-5, 5, -20));
    CubeMesh* cube3 = new CubeMesh(1, Vec3(5, 5, -30));
    CubeMesh* cube4 = new CubeMesh(10, Vec3(5, -5, -40));
    CubeMesh* cube5 = new CubeMesh(1, Vec3(-5, -5, 10));
    CubeMesh* cube6 = new CubeMesh(1, Vec3(-5, 5, 20));
    CubeMesh* cube7 = new CubeMesh(1, Vec3(5, 5, 30));
    CubeMesh* cube8 = new CubeMesh(10, Vec3(5, -5, 40));
    CubeMesh* cube9 = new CubeMesh(100, Vec3(0, 0, -400));

    cube1->color = Color(255, 0, 0);
    cube2->color = Color(255, 255, 0);
    cube3->color = Color(0, 255, 255);
    cube4->color = Color(255, 0, 255);
    cube5->color = Color(0, 0, 255);

    Mesh* mesh = LoadMeshFromOBJFile("Objects/Sphere.obj");
    mesh->scale = Vec3(500, 500, 500);
    mesh->position += World::forward * 1000;
    //mesh = LoadMeshFromOBJFile("Objects/tinker.obj");
    //Mesh* guitar = LoadMeshFromOBJFile("Objects/Guitar.obj");
    //guitar->position += (Camera::main->Forward() * 10) + Camera::main->Right();
    //Mesh* chair = LoadMeshFromOBJFile("Objects/Chair.obj");
    //chair->position += (Camera::main->Forward() * 10) + Camera::main->Left();
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
    window = glfwCreateWindow(800, 800, "Toast Engine", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    
    //glewInit();
    Init(window);
    
    glLineWidth(2);
    glPointSize(2);
    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);


        Time();
        Input(window);
        Physics(window);
        Draw();

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
