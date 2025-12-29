#include "App.h"

#include <stdexcept>
#include <glad/glad.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "Renderer/Shader.h"
#include "Renderer/Mesh.h"
#include "Renderer/Primitives.h"
#include "Camera/OrbitCamera.h"
#include "Robot/RobotScene.h"

static void glfw_error_callback(int error, const char* description) {
    (void)error; (void)description;
}

App::App() {
    glfwSetErrorCallback(glfw_error_callback);
    initGLFW();
    createWindow();
    initGLAD();
    initOpenGLState();
    initImGui();
}

App::~App() {
    shutdownImGui();
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}

void App::initGLFW() {
    if (!glfwInit()) throw std::runtime_error("GLFW init failed");

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
}

void App::createWindow() {
    window_ = glfwCreateWindow(1280, 720, "ArmViz", nullptr, nullptr);
    if (!window_) throw std::runtime_error("Window creation failed");

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
}

void App::initGLAD() {
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        throw std::runtime_error("GLAD init failed");
    }
}

void App::initOpenGLState() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void App::initImGui() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void App::shutdownImGui() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void App::beginFrame() {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    int w, h;
    glfwGetFramebufferSize(window_, &w, &h);
    glViewport(0, 0, w, h);

    glClearColor(0.05f, 0.06f, 0.09f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void App::endFrame() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window_);
}

void App::run() {
    // --- Shader sources (kept here, or move into a .glsl file later) ---
    const char* vsSrc = R"GLSL(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aColor;

        uniform mat4 uView;
        uniform mat4 uProj;
        uniform mat4 uModel;

        out vec3 vColor;

        void main() {
            vColor = aColor;
            gl_Position = uProj * uView * uModel * vec4(aPos, 1.0);
        }
    )GLSL";

    const char* fsSrc = R"GLSL(
        #version 330 core
        in vec3 vColor;
        out vec4 FragColor;
        uniform float uAlpha;
        void main() { FragColor = vec4(vColor, uAlpha); }
    )GLSL";

    Shader shader(vsSrc, fsSrc);

    // Geometry
    Mesh grid = Mesh::fromVertexColorLines(makeGridVerts(/*gridHalf=*/10));
    Mesh sphere = Mesh::fromVertexColorTriangles(makeSphereVerts(0.05f, 24, 16));

    // Camera + input
    OrbitCamera camera;
    camera.attachToWindow(window_);

    // Robot scene (origins.json + linkJoint + IK + ImGui panel)
    RobotScene robot("origins.json");

    bool rWasDown = false;

    while (!glfwWindowShouldClose(window_)) {
        beginFrame();

        ImGuiIO& io = ImGui::GetIO();

        // hotkeys not blocked by imgui keyboard: you can gate if you want
        bool rDown = glfwGetKey(window_, GLFW_KEY_R) == GLFW_PRESS;
        if (rDown && !rWasDown) camera.resetTarget();
        rWasDown = rDown;

        if (glfwGetKey(window_, GLFW_KEY_K) == GLFW_PRESS) {
            robot.reset();
        }

        // Update camera (mouse controls only if imgui not capturing mouse)
        camera.updateFromInput(io.WantCaptureMouse);

        // Matrices
        glm::mat4 view = camera.view();
        glm::mat4 proj = camera.orthoProjFromRadius(window_);

        shader.use();
        shader.setMat4("uView", view);
        shader.setMat4("uProj", proj);

        // Draw grid
        shader.setFloat("uAlpha", 1.0f);
        shader.setMat4("uModel", glm::mat4(1.0f));
        grid.drawLines();

        // Robot UI + IK solve + draw
        robot.uiAndSolve();
        robot.draw(shader, sphere);

        endFrame();
    }
}
