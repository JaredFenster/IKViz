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
#include "ImGuizmo/ImGuizmo.h"

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
    // --- Shaders ---
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
    Mesh grid = Mesh::fromVertexColorLines(makeGridVerts(10));
    Mesh sphere = Mesh::fromVertexColorTriangles(makeSphereVerts(0.05f, 24, 16));

    // Camera
    OrbitCamera camera;
    camera.attachToWindow(window_);

    // Robot
    RobotScene robot("origins.json");

    // Gizmo
    static ImGuizmoLite::Gizmo gizmo;
    static bool gizmoInit = false;

    std::vector<float> gizmoVerts;

    auto drawLine = [&](const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
        gizmoVerts.insert(gizmoVerts.end(), {a.x,a.y,a.z, c.x,c.y,c.z});
        gizmoVerts.insert(gizmoVerts.end(), {b.x,b.y,b.z, c.x,c.y,c.z});
    };

    bool rWasDown = false;
    static bool prevMouseDown = false;

    while (!glfwWindowShouldClose(window_)) {
        beginFrame();
        ImGuiIO& io = ImGui::GetIO();

        // Hotkeys
        bool rDown = glfwGetKey(window_, GLFW_KEY_R) == GLFW_PRESS;
        if (rDown && !rWasDown) camera.resetTarget();
        rWasDown = rDown;

        if (glfwGetKey(window_, GLFW_KEY_K) == GLFW_PRESS) {
            robot.reset();
            gizmoInit = false;
        }

        // Mouse
        double mx, my;
        glfwGetCursorPos(window_, &mx, &my);

        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);

        bool lmbDown = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        bool lmbPressed  = (lmbDown && !prevMouseDown);
        bool lmbReleased = (!lmbDown && prevMouseDown);
        prevMouseDown = lmbDown;

        // End effector pose (robot state)
        ImGuizmoLite::Pose ee{
            robot.eePos(),
            robot.eeRot()
        };

        // Initialize gizmo target ONCE from current EE
        if (!gizmoInit) {
            gizmo.setTarget(ee);
            gizmoInit = true;
        }

        // IMPORTANT: gizmo anchor pose is the TARGET, not the EE
        ImGuizmoLite::Pose gizmoPose = gizmo.target;

        glm::vec3 camPos = camera.getCamPos();
        glm::vec3 camForward = glm::normalize(gizmoPose.pos - camPos);

        // Update gizmo FIRST (picking/dragging happens around the target pose)
        gizmo.update(
            gizmoPose,
            camera.view(),
            camera.orthoProjFromRadius(window_),
            camPos,
            camForward,
            (float)mx, (float)my,
            (float)w, (float)h,
            lmbDown, lmbPressed, lmbReleased
        );

        // Camera only if gizmo not active
        if (!gizmo.capturingMouse) {
            camera.updateFromInput(io.WantCaptureMouse);
        }

        // Matrices AFTER camera update
        glm::mat4 view = camera.view();
        glm::mat4 proj = camera.orthoProjFromRadius(window_);

        shader.use();
        shader.setMat4("uView", view);
        shader.setMat4("uProj", proj);

        // IK target from gizmo (this is what robot tries to reach)
        robot.setIKTarget(gizmo.target.pos, gizmo.target.rot);
        robot.uiAndSolve();

        // Grid
        shader.setFloat("uAlpha", 1.0f);
        shader.setMat4("uModel", glm::mat4(1.0f));
        grid.drawLines();

        // Robot
        robot.draw(shader, sphere);

        // Gizmo draw AT TARGET
        gizmoVerts.clear();
        gizmo.draw(gizmo.target, proj, drawLine);

        if (!gizmoVerts.empty()) {
            Mesh gizmoMesh = Mesh::fromVertexColorLines(gizmoVerts);
            shader.setMat4("uModel", glm::mat4(1.0f));
            gizmoMesh.drawLines();
        }

        endFrame();
    }
}
