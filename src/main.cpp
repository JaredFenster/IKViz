#include <stdexcept>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <iostream>
#include "origin.h"
#include <fstream>
#include <nlohmann/json.hpp>
#include "linkJoint.h"
#include "IK.h"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>



//Camera variables

static bool g_orbiting = false;
static double g_lastX = 0.0, g_lastY = 0.0;
static float g_yaw = 0.7854f;
static float g_pitch = 0.35f; //Radians
static float g_radius = 3.0f;
static glm::vec3 g_target(0.0f, 0.0f, 0.0f);
static bool g_panning = false;
static double g_panLastX = 0.0, g_panLastY = 0.0;
static Origin* end;

static bool rWasDown = false;

static void scroll_callback(GLFWwindow*, double /*xoffset*/, double yoffset){
    //Zoom
    g_radius *= (yoffset > 0.0) ? 0.9f : 1.1f;
    if (g_radius < 0.3f) g_radius = 0.3f;
    if (g_radius > 50.0f) g_radius = 50.0f;
}

static void glfw_error_callback(int error, const char* description){
    // You can set a breakpoint here if you want
    (void)error;
    (void)description;
}

static GLuint compileShader(GLenum type, const char* src)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok)
    {
        GLint len = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
        std::string log(len, '\0');
        glGetShaderInfoLog(shader, len, nullptr, log.data());
        glDeleteShader(shader);
        throw std::runtime_error("Shader compile failed:\n" + log);
    }
    return shader;
}

static GLuint linkProgram(GLuint vs, GLuint fs)
{
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);

    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok)
    {
        GLint len = 0;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &len);
        std::string log(len, '\0');
        glGetProgramInfoLog(prog, len, nullptr, log.data());
        glDeleteProgram(prog);
        throw std::runtime_error("Program link failed:\n" + log);
    }
    return prog;
}

//------------------------------main------------------------------------------------------------------------------------------------------

int main(){
    glfwSetErrorCallback(glfw_error_callback);

    //Start GLFW
    if (!glfwInit())
        throw std::runtime_error("GLFW init failed");

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    //Create a window
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
    GLFWwindow* window = glfwCreateWindow(1280, 720, "ArmViz", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("Window creation failed");
    }

    glfwSetScrollCallback(window, scroll_callback);

    //Make its OpenGL context current
    glfwMakeContextCurrent(window);

    //vsync
    glfwSwapInterval(1);

    //Load OpenGL functions (must happen AFTER context is current)
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwDestroyWindow(window);
        glfwTerminate();
        throw std::runtime_error("GLAD init failed");
    }

    glEnable(GL_DEPTH_TEST);

    // -------------------- ImGui init --------------------
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    // Optional: style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");


    //Setup 

    const char* vsSrc = R"GLSL(
                    #version 330 core
                    layout (location = 0) in vec3 aPos;
                    layout (location = 1) in vec3 aColor;

                    uniform mat4 uView;
                    uniform mat4 uProj;
                    uniform mat4 uModel;

                    out vec3 vColor;

                    void main()
                    {
                        vColor = aColor;
                        gl_Position = uProj * uView * uModel * vec4(aPos, 1.0);
                    }
                    )GLSL";

    const char* fsSrc = R"GLSL(
                        #version 330 core
                        in vec3 vColor;
                        out vec4 FragColor;

                        void main()
                        {
                            FragColor = vec4(vColor, 1.0);
                        }
                        )GLSL";




    //CREATE AXIS & GRID
    const int gridHalf = 10;

    std::vector<float> gridVerts;
    gridVerts.reserve(( (2*gridHalf + 1) * 2 /*two endpoints*/ * 2 /*x-lines + y-lines*/ * 6));

    //helper
    auto pushV = [&](float x, float y, float z, float r, float g, float b) {
        gridVerts.push_back(x); gridVerts.push_back(y); gridVerts.push_back(z);
        gridVerts.push_back(r); gridVerts.push_back(g); gridVerts.push_back(b);
    };


    //X
    pushV((float)-gridHalf, 0, 0, 1, 0, 0);
    pushV((float)+gridHalf, 0, 0, 1, 0, 0);

    //Y
    pushV(0, (float)-gridHalf, 0, 0, 0, 1);
    pushV(0, (float)+gridHalf, 0, 0, 0, 1);

    //Z
    pushV(0, 0, (float)-0, 0, 1, 0);
    pushV(0, 0, (float)+2, 0, 1, 0);
    

    for (int i = -gridHalf; i <= gridHalf; i++) {
        if(i == 0) continue;

        float r = 0.25f;
        float g = 0.25f;
        float b = 0.25f;

        float x = (float)i;
        float y = (float)i;

        // Lines parallel to Y at each X: (x, -gridHalf, 0) -> (x, +gridHalf, 0)
        pushV(x, (float)-gridHalf, 0.0f, r, g, b);
        pushV(x, (float)+gridHalf, 0.0f, r, g, b);

        // Lines parallel to X at each Y: (-gridHalf, y, 0) -> (+gridHalf, y, 0)
        pushV((float)-gridHalf, y, 0.0f, r, g, b);
        pushV((float)+gridHalf, y, 0.0f, r, g, b);
    }



    GLuint gridVAO = 0, gridVBO = 0;
    glGenVertexArrays(1, &gridVAO);
    glGenBuffers(1, &gridVBO);
    
    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(gridVerts.size() * sizeof(float)),
                 gridVerts.data(),
                 GL_STATIC_DRAW);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    GLsizei gridVertexCount = (GLsizei)(gridVerts.size() / 6); // 6 floats per vertex

    //CREATE ROTATE SPHERE

    float sphereRadius = 0.05f;
    int slices = 24;
    int stacks = 16;

    std::vector<float> sphereVerts;
    
    auto pushSphereV = [&](float x, float y, float z) {
        sphereVerts.push_back(x);
        sphereVerts.push_back(y);
        sphereVerts.push_back(z);
        sphereVerts.push_back(1.0f); // yellow
        sphereVerts.push_back(1.0f);
        sphereVerts.push_back(0.0f);
    };
    for (int i = 0; i < stacks; ++i){
        float phi0 = glm::pi<float>() * (float)i / stacks;
        float phi1 = glm::pi<float>() * (float)(i + 1) / stacks;
    
        for (int j = 0; j < slices; ++j){
            float theta0 = glm::two_pi<float>() * (float)j / slices;
            float theta1 = glm::two_pi<float>() * (float)(j + 1) / slices;
        
            // 4 corners of quad
            glm::vec3 p0(
                sphereRadius * sin(phi0) * cos(theta0),
                sphereRadius * sin(phi0) * sin(theta0),
                sphereRadius * cos(phi0)
            );
        
            glm::vec3 p1(
                sphereRadius * sin(phi0) * cos(theta1),
                sphereRadius * sin(phi0) * sin(theta1),
                sphereRadius * cos(phi0)
            );
        
            glm::vec3 p2(
                sphereRadius * sin(phi1) * cos(theta0),
                sphereRadius * sin(phi1) * sin(theta0),
                sphereRadius * cos(phi1)
            );
        
            glm::vec3 p3(
                sphereRadius * sin(phi1) * cos(theta1),
                sphereRadius * sin(phi1) * sin(theta1),
                sphereRadius * cos(phi1)
            );
        
            // triangle 1
            pushSphereV(p0.x, p0.y, p0.z);
            pushSphereV(p2.x, p2.y, p2.z);
            pushSphereV(p1.x, p1.y, p1.z);
        
            // triangle 2
            pushSphereV(p1.x, p1.y, p1.z);
            pushSphereV(p2.x, p2.y, p2.z);
            pushSphereV(p3.x, p3.y, p3.z);
        }
    }

    GLuint sphereVAO = 0, sphereVBO = 0;
    glGenVertexArrays(1, &sphereVAO);
    glGenBuffers(1, &sphereVBO);

    glBindVertexArray(sphereVAO);
    glBindBuffer(GL_ARRAY_BUFFER, sphereVBO);
    glBufferData(GL_ARRAY_BUFFER,
                 sphereVerts.size() * sizeof(float),
                 sphereVerts.data(),
                 GL_STATIC_DRAW);

    // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    GLsizei sphereVertexCount = (GLsizei)(sphereVerts.size() / 6);


    //OTHER STUFF

    GLuint vs = compileShader(GL_VERTEX_SHADER, vsSrc);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fsSrc);
    GLuint program = linkProgram(vs, fs);
    glDeleteShader(vs);
    glDeleteShader(fs);
    
    glm::mat4 view(1.0f);
    glm::mat4 proj(1.0f);

    GLint uViewLoc = glGetUniformLocation(program, "uView");
    GLint uProjLoc = glGetUniformLocation(program, "uProj");
    GLint uModelLoc = glGetUniformLocation(program, "uModel");




    //just declare these once
    glm::vec3 up(0.0f, 0.0f, 1.0f);
    glm::vec3 forward(cosf(g_yaw) * cosf(g_pitch), sinf(g_yaw) * cosf(g_pitch), sinf(g_pitch));
    bool renderSphere = false;

//------------------------robot-stuff--------------------------------------------------------------------------------


    std::vector<Origin> origins;

    std::ifstream file("origins.json");
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open origins.json");
    }

    nlohmann::json j;
    file >> j;

    const auto& arr = j.at("origins");
    origins.reserve(arr.size());

    for (const auto& o : arr) {
        glm::vec3 pos(
            o.at("position").at(0).get<float>(),
            o.at("position").at(1).get<float>(),
            o.at("position").at(2).get<float>()
        );

        int xDir = o.at("xDir").get<int>();
        int zDir = o.at("zDir").get<int>();
        float axisLength = o.value("axisLength", 0.2f);
        int rotateAxis = o.value("rotateAxis", 3);
        float angleDeg = o.value("angleDeg", 0.0f);

        origins.emplace_back(pos, xDir, zDir, axisLength, rotateAxis, angleDeg);

        // optional debug
        std::cout << "Loaded "
                  << o.value("name", std::string("unnamed"))
                  << " at (" << pos.x << "," << pos.y << "," << pos.z << ")\n";
    }

    std::cout << "origins loaded = " << origins.size() << "\n";


    for(int i = 0; i < origins.size(); i++){
        origins[i].setAxisRadius(0.015f);
        origins[i].setAxisLength(0.25f);
        origins[i].setCylinderSlices(20);
    }




    origins.reserve(j["origins"].size());

    //link
    linkJoint robot(origins.at(0));
    robot.addJoint(origins.at(1));
    robot.addJoint(origins.at(2));
    robot.addJoint(origins.at(3));
    end = &origins.at(3);
    robot.setLinkRadius(0.05f);
    robot.setLinkSlices(18);

    IK ik(robot);
    static glm::vec3 targetWorld(0.2f, 0.1f, 0.25f);

    




//------------------------Main-Loop--------------------------------------------------------------------------------



    while (!glfwWindowShouldClose(window))
    {   
        glfwPollEvents();
        // -------------------- ImGui frame start --------------------
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGuiIO& io = ImGui::GetIO(); // refresh each frame

        //Resize handling: tell OpenGL the window size
        int w, h;
        glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h);

        float aspect = (h == 0) ? 1.0f : (float)w / (float)h;

        float orthoSize = g_radius;
        float halfH = orthoSize;
        float halfW = orthoSize * aspect;

        // Perspective projection
        proj = glm::ortho(-halfW, halfW, -halfH, halfH, 200.0f, -200.0f);


        //panning basis
        glm::vec3 target = g_target;
        glm::vec3 camPos = target - forward * g_radius;

        glm::vec3 camForward = glm::normalize(target - camPos);         
        glm::vec3 camRight   = glm::normalize(glm::cross(camForward, up));
        glm::vec3 camUp      = glm::normalize(glm::cross(camRight, camForward));


        //Blank Screen
        glClearColor(0.05f, 0.06f, 0.09f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        bool rDown = glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS;
        if(rDown && !rWasDown) g_target = glm::vec3(0);
        rWasDown = rDown;


        if(glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS){
            robot.reset();
        }
        //if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){
        //    robot.rotateJoint(0, 5);
        //}
        //if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        //    robot.rotateJoint(1, 5);
        //}
        //if(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
        //    robot.rotateJoint(2, 5);
        //}
        //if(glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS){
        //    robot.rotateJoint(3, 5);
        //}



        // -------------------- IK + UI controls --------------------
        static bool  ikEnabled     = true;
        static int   itersPerFrame = 2;
        static float lambda        = 0.15f;
        static float maxStepDeg    = 2.0f;

        ImGui::Begin("IK Controls");
        ImGui::Checkbox("Enable IK", &ikEnabled);
        ImGui::DragFloat3("Target (world)", &targetWorld.x, 0.01f);
        ImGui::SliderInt("Iterations / frame", &itersPerFrame, 1, 30);
        ImGui::SliderFloat("Damping (lambda)", &lambda, 0.01f, 1.0f);
        ImGui::SliderFloat("Max step (deg)", &maxStepDeg, 0.1f, 10.0f);

        if (ImGui::Button("Reset Robot (K)")) {
            robot.reset();
        }
        ImGui::SameLine();
        if (ImGui::Button("Target = End Effector")) {
            targetWorld = end->getPos(); // uses your global end pointer
        }

        ImGui::Text("EE: (%.3f, %.3f, %.3f)", end->getPos().x, end->getPos().y, end->getPos().z);
        ImGui::End();

        // Smooth IK update (a few iterations per frame)
        if (ikEnabled) {
            ik.solvePosition(
                targetWorld,
                -1,
                itersPerFrame,
                1e-3f,
                lambda,
                maxStepDeg
            );
        }



        if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS){
            std::cout << "End Effector Pos: (" << end->getX() << ", " << end->getY() << ", " << end->getZ() <<")"<< std::endl;
        }


        // Only allow camera controls if ImGui is NOT capturing the mouse
        if (!io.WantCaptureMouse) {
        
            //Panning CTRL + MIDDLE
            if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS &&
                glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
            {
                double x, y;
                glfwGetCursorPos(window, &x, &y);
            
                if (!g_panning) {
                    g_panning = true;
                    g_panLastX = x; g_panLastY = y;
                }
            
                double dx = x - g_panLastX;
                double dy = y - g_panLastY;
                g_panLastX = x; g_panLastY = y;
            
                float panSpeed = g_radius * 0.0025f;
                g_target += (-camRight * (float)dx + camUp * (float)dy) * panSpeed;
                renderSphere = true;
            }
            //Orbit MIDDLE
            else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS)
            {
                double x, y;
                glfwGetCursorPos(window, &x, &y);
            
                if (!g_orbiting) {
                    g_orbiting = true;
                    g_lastX = x; g_lastY = y;
                }
            
                double dx = x - g_lastX;
                double dy = y - g_lastY;
                g_lastX = x; g_lastY = y;
            
                const float sensitivity = 0.005f;
                g_yaw   += (float)dx * sensitivity;
                g_pitch += (float)(dy) * sensitivity;
            
                const float limit = 1.55334f;
                if (g_pitch >  limit) g_pitch =  limit;
                if (g_pitch < -limit) g_pitch = -limit;
            
                renderSphere = true;
            }
            else {
                g_panning = false;
                g_orbiting = false;
            }
        
        } else {
            // If ImGui is using the mouse, stop camera drags cleanly
            g_panning = false;
            g_orbiting = false;
        }




        target = g_target;
        forward = glm::vec3( cosf(g_yaw) * cosf(g_pitch), sinf(g_yaw) * cosf(g_pitch), sinf(g_pitch));
        camPos = target - forward * g_radius;
        view = glm::lookAt(camPos, target, up);


        glUseProgram(program);
        glUniformMatrix4fv(uViewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(uProjLoc, 1, GL_FALSE, glm::value_ptr(proj));

        // grid
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_FALSE); 

        glm::mat4 I(1.0f);
        glUniformMatrix4fv(uModelLoc, 1, GL_FALSE, glm::value_ptr(I));
        glBindVertexArray(gridVAO);
        glDrawArrays(GL_LINES, 0, gridVertexCount);

        glDepthMask(GL_TRUE); 

        robot.verts.clear();
        robot.addVerts();
        robot.setupBuffers();

        ik.solvePosition(
        targetWorld,
        -1,      // end effector = last joint
        2,       // only 2 iterations per frame
        1e-3f,   // tolerance
        0.15f,   // damping (increase if jitter)
        2.0f     // max step deg (smaller = smoother)
        );
        
        
        // Sphere
        if(renderSphere){
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);

            // don’t let the wireframe sphere write depth
            glDepthMask(GL_FALSE);

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glm::mat4 sphereModel(1.0f);
            sphereModel = glm::translate(sphereModel, glm::vec3(target));
            glUniformMatrix4fv(uModelLoc, 1, GL_FALSE, glm::value_ptr(sphereModel));
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glBindVertexArray(sphereVAO);
            glDrawArrays(GL_TRIANGLES, 0, sphereVertexCount);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            renderSphere = false;

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            // restore normal depth writing
            glDepthMask(GL_TRUE);
        } 



        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        // don’t let the wireframe sphere write depth
        glDepthMask(GL_FALSE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glm::mat4 sphereModel(1.0f);
        sphereModel = glm::translate(sphereModel, glm::vec3(targetWorld));
        glUniformMatrix4fv(uModelLoc, 1, GL_FALSE, glm::value_ptr(sphereModel));
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glBindVertexArray(sphereVAO);
        glDrawArrays(GL_TRIANGLES, 0, sphereVertexCount);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        renderSphere = false;
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        // restore normal depth writing
        glDepthMask(GL_TRUE);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);  
        glDepthMask(GL_TRUE);

        

        //links
        robot.link(program, uModelLoc);


        //Origins
        for (const Origin& o : origins) {
            o.draw(program, uModelLoc);
        }
        
        glDisable(GL_CULL_FACE);


        glBindVertexArray(0);
        //Show the frame
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    //----------------------------Cleanup-----------------------------------------------------------------------------------------
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glDeleteBuffers(1, &gridVBO);
    glDeleteVertexArrays(1, &gridVAO);
    glDeleteProgram(program);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
