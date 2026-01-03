// App.cpp
#include "App.h"
#define GLM_ENABLE_EXPERIMENTAL

#include <iostream>
#include <stdexcept>
#include <glad/glad.h>
#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>
#include <cmath>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "Renderer/Shader.h"
#include "Renderer/Mesh.h"
#include "Renderer/Primitives.h"
#include "Camera/OrbitCamera.h"
#include "Robot/RobotScene.h"
#include "Robot/URDFRobot.h"
#include "ImGuizmo/ImGuizmo.h"
#include "InverseKinematics/IK.h"
#include "Trajectory/Trajectory.h"

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

static std::string pickExistingPath(const std::string& a, const std::string& b) {
    if (std::filesystem::exists(a)) return a;
    if (std::filesystem::exists(b)) return b;
    return a;
}

void App::run() {
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

        uniform bool uUseUniformColor;
        uniform vec3 uUniformColor;

        void main() {
            vec3 col = uUseUniformColor ? uUniformColor : vColor;
            FragColor = vec4(col, uAlpha);
        }
    )GLSL";

    Shader shader(vsSrc, fsSrc);

    Mesh grid    = Mesh::fromVertexColorLines(makeGridVerts(10));
    Mesh unitCyl = Mesh::fromVertexColorTriangles(makeCylinderVerts(1.0f, 1.0f, 24));

    OrbitCamera camera;
    camera.attachToWindow(window_);

    // ===========================
    // TWO ROBOTS (same URDF)
    //  - robotA: controlled by gizmo/IK
    //  - robotB: jogged along trajectory using its OWN chain
    // ===========================
    RobotScene robotA;
    RobotScene robotB;

    bool robotLoadedA = false;
    bool robotLoadedB = false;
    std::string robotErrA;
    std::string robotErrB;

    std::string urdfPath   = pickExistingPath("robot/your_robot.urdf", "build/robot/your_robot.urdf");
    std::string meshesRoot = pickExistingPath("robot/meshes",         "build/robot/meshes");

    // IMPORTANT: chain must be built PER-robot instance
    URDFIK::ChainInfo chainA;
    URDFIK::ChainInfo chainB;
    bool chainBuiltA = false;
    bool chainBuiltB = false;

    auto loadRobots = [&]() {
        // reset
        robotLoadedA = robotLoadedB = false;
        robotErrA.clear(); robotErrB.clear();
        chainBuiltA = chainBuiltB = false;
        chainA = URDFIK::ChainInfo{};
        chainB = URDFIK::ChainInfo{};

        // robotA
        try {
            robotLoadedA = robotA.LoadURDF(urdfPath, meshesRoot);
            if (!robotLoadedA) {
                robotErrA = "RobotScene::LoadURDF returned false (paths or parse failed).";
            } else {
                chainA = URDFIK::BuildSerialChain(robotA.Robot());
                chainBuiltA = !chainA.jointIdx.empty();
                if (!chainBuiltA) robotErrA = "Loaded robotA, but failed to build a serial joint chain.";
            }
        } catch (const std::exception& e) {
            robotLoadedA = false;
            robotErrA = e.what();
            chainBuiltA = false;
        }

        // robotB
        try {
            robotLoadedB = robotB.LoadURDF(urdfPath, meshesRoot);
            if (!robotLoadedB) {
                robotErrB = "RobotScene::LoadURDF returned false (paths or parse failed).";
            } else {
                chainB = URDFIK::BuildSerialChain(robotB.Robot());
                chainBuiltB = !chainB.jointIdx.empty();
                if (!chainBuiltB) robotErrB = "Loaded robotB, but failed to build a serial joint chain.";
            }
        } catch (const std::exception& e) {
            robotLoadedB = false;
            robotErrB = e.what();
            chainBuiltB = false;
        }
    };

    loadRobots();

    // Gizmo
    static ImGuizmoLite::Gizmo gizmo;
    static bool gizmoInit = false;

    struct GizmoCyl { glm::vec3 a,b; float r; glm::vec3 c; };
    std::vector<GizmoCyl> gizmoCyls;

    auto drawCylinder = [&](const glm::vec3& a, const glm::vec3& b, float radius, const glm::vec3& color) {
        gizmoCyls.push_back({a, b, radius, color});
    };

    auto cylinderModel = [&](const glm::vec3& a, const glm::vec3& b, float radius) -> glm::mat4 {
        glm::vec3 d = b - a;
        float len = glm::length(d);
        if (len < 1e-6f) return glm::mat4(1.0f);

        glm::vec3 z = d / len;
        glm::vec3 up = (std::fabs(z.y) < 0.99f) ? glm::vec3(0,1,0) : glm::vec3(1,0,0);
        glm::vec3 x = glm::normalize(glm::cross(up, z));
        glm::vec3 y = glm::cross(z, x);

        glm::mat4 R(1.0f);
        R[0] = glm::vec4(x, 0.0f);
        R[1] = glm::vec4(y, 0.0f);
        R[2] = glm::vec4(z, 0.0f);

        glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(radius, radius, len));
        glm::mat4 T = glm::translate(glm::mat4(1.0f), a);

        return T * R * S;
    };

    bool rWasDown = false;
    static bool prevMouseDown = false;

    // IK controls
    static bool ikEnabled = true;
    static bool ikSolveEveryFrame = false;
    static bool ikUseOrientation = true;

    static int   ikMaxIter = 60;
    static float ikPosTol = 0.002f;
    static float ikRotTol = 0.02f;
    static float ikMaxStepDeg = 6.0f;
    static float ikRotWeight = 1.0f;
    static float ikLambda = 0.20f;

    // Jog controls
    static int density = 1000;
    static int jogIndex = 0;
    static bool jogging = false;
    static Trajectory traj;

    // Optional: how many trajectory points to advance per frame (speed)
    static int jogStride = 1;

    // Jog settings
    static bool jogInterpRotation = true;

    // Frozen jog endpoints (so moving the gizmo mid-jog doesn't change the goal)
    static glm::vec3 jogEndPos(0.0f);
    static glm::quat jogEndRot(1,0,0,0);

    while (!glfwWindowShouldClose(window_)) {
        beginFrame();
        ImGuiIO& io = ImGui::GetIO();

        bool rDown = glfwGetKey(window_, GLFW_KEY_R) == GLFW_PRESS;
        if (rDown && !rWasDown) camera.resetTarget();
        rWasDown = rDown;

        // --- cursor + sizes (DPI-safe) ---
        double mxWin, myWin;
        glfwGetCursorPos(window_, &mxWin, &myWin);

        int winW, winH;
        glfwGetWindowSize(window_, &winW, &winH);

        int fbW, fbH;
        glfwGetFramebufferSize(window_, &fbW, &fbH);

        float sx = (winW > 0) ? (float)fbW / (float)winW : 1.0f;
        float sy = (winH > 0) ? (float)fbH / (float)winH : 1.0f;

        // IMPORTANT: keep top-left origin (NO Y FLIP) because Gizmo.h does the flip internally
        float mx = (float)mxWin * sx;
        float my = (float)myWin * sy;

        bool lmbDown = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        bool lmbPressed  = (lmbDown && !prevMouseDown);
        bool lmbReleased = (!lmbDown && prevMouseDown);
        prevMouseDown = lmbDown;

        // init gizmo once (snap to robotA EE)
        if (!gizmoInit && robotLoadedA && chainBuiltA) {
            std::vector<glm::mat4> jf;
            URDFIK::FKResult fk = URDFIK::ComputeFK(robotA.Robot(), chainA, jf);
            gizmo.target.pos = fk.pos;
            gizmo.target.rot = fk.rot;
            gizmoInit = true;
        }

        if (!gizmo.capturingMouse) {
            camera.updateFromInput(io.WantCaptureMouse);
        }

        // Build view/proj AFTER camera update so gizmo math == render math
        glm::mat4 view = camera.view();
        glm::mat4 proj = camera.orthoProjFromRadius(window_);
        glm::vec3 camPos = camera.getCamPos();

        // Use actual camera forward from the view matrix (stable for ortho/persp)
        glm::vec3 camForward = glm::normalize(-glm::vec3(view[2]));

        // --- Update gizmo using the SAME matrices you'll render with ---
        gizmo.update(
            gizmo.target,   // gizmo is anchored on target
            view,
            proj,
            camPos,
            camForward,
            mx, my,
            (float)fbW, (float)fbH,
            lmbDown, lmbPressed, lmbReleased
        );
        gizmo.target.rot = glm::normalize(gizmo.target.rot);

        // Then render using *these same* view/proj (DO NOT recompute later)
        shader.use();
        shader.setMat4("uView", view);
        shader.setMat4("uProj", proj);

        shader.setBool("uUseUniformColor", false);
        shader.setFloat("uAlpha", 1.0f);

        // ---- UI ----
        ImGui::Begin("ArmViz");

        ImGui::Text("URDF: %s", urdfPath.c_str());
        ImGui::Text("Meshes: %s", meshesRoot.c_str());

        if (!robotLoadedA || !robotLoadedB) {
            ImGui::Separator();
            if (!robotLoadedA) {
                ImGui::TextColored(ImVec4(1,0.35f,0.35f,1), "robotA NOT loaded");
                ImGui::TextWrapped("%s", robotErrA.c_str());
            }
            if (!robotLoadedB) {
                ImGui::TextColored(ImVec4(1,0.35f,0.35f,1), "robotB NOT loaded");
                ImGui::TextWrapped("%s", robotErrB.c_str());
            }
            if (ImGui::Button("Retry Load")) loadRobots();
        } else {
            if (ImGui::Button("Reload URDF (both)")) {
                gizmoInit = false; // re-snap after reload
                loadRobots();
            }

            ImGui::Separator();
            ImGui::Text("IK (Hierarchical DLS Pose) -> robotA only");
            ImGui::Checkbox("Enable IK", &ikEnabled);
            ImGui::SameLine();
            ImGui::Checkbox("Solve every frame", &ikSolveEveryFrame);
            ImGui::Checkbox("Use orientation", &ikUseOrientation);

            ImGui::SliderInt("Max iters", &ikMaxIter, 1, 200);
            ImGui::SliderFloat("Pos tol (m)", &ikPosTol, 0.0005f, 0.02f, "%.4f");
            ImGui::SliderFloat("Rot tol (rad)", &ikRotTol, 0.002f, 0.2f, "%.3f");
            ImGui::SliderFloat("Max step (deg)", &ikMaxStepDeg, 0.5f, 15.0f, "%.1f");
            ImGui::SliderFloat("Rot weight", &ikRotWeight, 0.0f, 3.0f, "%.2f");
            ImGui::SliderFloat("Damping (lambda)", &ikLambda, 0.001f, 2.0f, "%.3f");

            ImGui::Separator();
            ImGui::Text("Jog:");
            ImGui::SliderInt("Jog Density", &density, 100, 5000);
            ImGui::SliderInt("Jog Stride (pts/frame)", &jogStride, 1, 50);
            ImGui::Checkbox("Interpolate Rotation (slerp)", &jogInterpRotation);

            if (chainBuiltA) {
                if (ImGui::Button("Snap gizmo to robotA EE (pos+rot)")) {
                    std::vector<glm::mat4> jf;
                    URDFIK::FKResult fk = URDFIK::ComputeFK(robotA.Robot(), chainA, jf);
                    gizmo.target.pos = fk.pos;
                    gizmo.target.rot = fk.rot;
                }
            } else {
                ImGui::TextColored(ImVec4(1,0.6f,0.2f,1), "Chain not built (robotA): %s", robotErrA.c_str());
            }

            if (!chainBuiltB) {
                ImGui::TextColored(ImVec4(1,0.6f,0.2f,1), "Chain not built (robotB): %s", robotErrB.c_str());
            }

            ImGui::Separator();
            ImGui::Text("Joints (radians) [editing robotA]:");

            // Start jog: generate trajectory from robotB EE -> gizmo target (pos + rot)
            if (ImGui::Button("JOG ROBOT") && !jogging) {
                if (robotLoadedB && chainBuiltB) {
                    jogging = true;
                    jogIndex = 0;

                    // freeze endpoint at the moment jog begins
                    jogEndPos = gizmo.target.pos;
                    jogEndRot = gizmo.target.rot;

                    std::vector<glm::mat4> jfB;
                    URDFIK::FKResult fkB = URDFIK::ComputeFK(robotB.Robot(), chainB, jfB);

                    if (jogInterpRotation) {
                        traj.GeneratePoses(
                            fkB.pos, fkB.rot,
                            jogEndPos, jogEndRot,
                            density
                        );
                        std::cout << "Jog Started (pos+rot)\n";
                        std::cout << traj.getNumPoses() << " Poses Generated\n";
                    } else {
                        traj.GeneratePoints(fkB.pos, jogEndPos, density);
                        std::cout << "Jog Started (pos only)\n";
                        std::cout << traj.getNumPoints() << " Points Generated\n";
                    }
                }
            }

            auto& urdfA = robotA.Robot();
            for (const auto& j : urdfA.Joints()) {
                if (j.type == JointType::Fixed) continue;

                float q = urdfA.GetJointAngle(j.name);
                float lo = j.hasLimits ? j.lower : -3.14159f;
                float hi = j.hasLimits ? j.upper :  3.14159f;

                if (ImGui::SliderFloat(j.name.c_str(), &q, lo, hi)) {
                    urdfA.SetJointAngle(j.name, q);
                }
            }
        }

        ImGui::Separator();
        ImGui::Text("Target:");
        ImGui::Text("pos: %.3f %.3f %.3f", gizmo.target.pos.x, gizmo.target.pos.y, gizmo.target.pos.z);
        ImGui::End();

        // ---- IK solve (robotA only) ----
        if (robotLoadedA && chainBuiltA && ikEnabled) {
            bool shouldSolve = ikSolveEveryFrame || gizmo.capturingMouse;
            if (shouldSolve) {
                if (ikUseOrientation) {
                    (void)URDFIK::SolvePoseHierDLS(
                        robotA.Robot(),
                        chainA,
                        gizmo.target.pos,
                        gizmo.target.rot,
                        ikMaxIter,
                        ikPosTol,
                        ikRotTol,
                        ikMaxStepDeg,
                        ikRotWeight,
                        ikLambda
                    );
                } else {
                    (void)URDFIK::SolvePoseHierDLS(
                        robotA.Robot(),
                        chainA,
                        gizmo.target.pos,
                        gizmo.target.rot,
                        ikMaxIter,
                        ikPosTol,
                        999.0f,
                        ikMaxStepDeg,
                        0.0f,
                        ikLambda
                    );
                }
            }
        }

        // ---- Jog solve (robotB only, using chainB) ----
        if (jogging) {
            if (!robotLoadedB || !chainBuiltB || !ikEnabled) {
                jogging = false;
                jogIndex = 0;
            } else {
                if (jogInterpRotation) {
                    const int n = traj.getNumPoses();
                    if (n < 2) {
                        jogging = false;
                        jogIndex = 0;
                    } else {
                        jogIndex = std::clamp(jogIndex, 0, n - 1);

                        glm::vec3 jogPos = traj.getPos(jogIndex);
                        glm::quat jogRot = traj.getRot(jogIndex);

                        // If user disabled orientation globally, still jog position only.
                        float rotTolUse = ikUseOrientation ? ikRotTol : 999.0f;
                        float rotWUse   = ikUseOrientation ? ikRotWeight : 0.0f;

                        (void)URDFIK::SolvePoseHierDLS(
                            robotB.Robot(), chainB,
                            jogPos, jogRot,
                            ikMaxIter, ikPosTol, rotTolUse,
                            ikMaxStepDeg, rotWUse, ikLambda
                        );

                        jogIndex += std::max(1, jogStride);
                        if (jogIndex >= n) {
                            jogIndex = 0;
                            jogging = false;
                        }
                    }
                } else {
                    const int n = traj.getNumPoints();
                    if (n < 2) {
                        jogging = false;
                        jogIndex = 0;
                    } else {
                        jogIndex = std::clamp(jogIndex, 0, n - 1);

                        glm::vec3 jogPos = traj.getPoint(jogIndex);

                        // keep orientation fixed to the frozen endpoint, or ignore if orientation disabled
                        float rotTolUse = ikUseOrientation ? ikRotTol : 999.0f;
                        float rotWUse   = ikUseOrientation ? ikRotWeight : 0.0f;

                        (void)URDFIK::SolvePoseHierDLS(
                            robotB.Robot(), chainB,
                            jogPos, jogEndRot,
                            ikMaxIter, ikPosTol, rotTolUse,
                            ikMaxStepDeg, rotWUse, ikLambda
                        );

                        jogIndex += std::max(1, jogStride);
                        if (jogIndex >= n) {
                            jogIndex = 0;
                            jogging = false;
                        }
                    }
                }
            }
        }

        // ---- Draw grid ----
        shader.setMat4("uModel", glm::mat4(1.0f));
        grid.drawLines();

        // ---- Draw BOTH robots ----
        if (robotLoadedA) robotA.Draw(shader);
        if (robotLoadedB) robotB.Draw(shader);

        // ---- Draw gizmo at target ----
        gizmoCyls.clear();
        gizmo.draw(gizmo.target, proj, drawCylinder);

        shader.setBool("uUseUniformColor", true);
        shader.setFloat("uAlpha", 1.0f);

        for (const auto& seg : gizmoCyls) {
            shader.setVec3("uUniformColor", seg.c);
            shader.setMat4("uModel", cylinderModel(seg.a, seg.b, seg.r));
            unitCyl.drawTriangles();
        }

        shader.setBool("uUseUniformColor", false);

        endFrame();
    }
}
