#include "App.h"
#define GLM_ENABLE_EXPERIMENTAL

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
#include <glm/gtx/quaternion.hpp>     // quat_cast, angleAxis
#include <glm/gtc/constants.hpp>

#include "Renderer/Shader.h"
#include "Renderer/Mesh.h"
#include "Renderer/Primitives.h"
#include "Camera/OrbitCamera.h"
#include "Robot/RobotScene.h"
#include "Robot/URDFRobot.h"
#include "Robot/URDFMath.h"           // urdfXYZRPY
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

static std::string pickExistingPath(const std::string& a, const std::string& b) {
    if (std::filesystem::exists(a)) return a;
    if (std::filesystem::exists(b)) return b;
    return a;
}

// ---------------- URDF chain + FK helpers ----------------

struct ChainInfo {
    std::vector<int> jointIdx;  // indices into urdf.Joints()
    std::string eeLink;
};

static ChainInfo buildSerialChain(const URDFRobot& urdf) {
    ChainInfo c;
    std::string cur = urdf.RootLink();

    for (int safety = 0; safety < 64; ++safety) {
        const auto& childs = urdf.ChildJointsOf(cur);
        if (childs.empty()) {
            c.eeLink = cur;
            break;
        }
        int jIdx = childs[0]; // serial chain: first child joint
        c.jointIdx.push_back(jIdx);
        cur = urdf.Joints()[jIdx].childLink;
    }

    if (c.eeLink.empty()) c.eeLink = cur;
    return c;
}

struct FKResult {
    glm::vec3 pos{0};
    glm::quat rot{1,0,0,0};
};

// Computes:
// - outJointFrameWorld[k] = world transform of joint k frame (after origin, before rotation)
// - returns EE pose (at last child link frame)
static FKResult computeFK(
    const URDFRobot& urdf,
    const ChainInfo& chain,
    std::vector<glm::mat4>& outJointFrameWorld
) {
    outJointFrameWorld.clear();
    outJointFrameWorld.reserve(chain.jointIdx.size());

    glm::mat4 T = glm::mat4(1.0f);

    for (int idx : chain.jointIdx) {
        const URDFJoint& j = urdf.Joints()[idx];

        glm::mat4 T_origin = urdfXYZRPY(j.originXyz, j.originRpy);

        glm::mat4 T_jointFrame = T * T_origin;
        outJointFrameWorld.push_back(T_jointFrame);

        float q = urdf.GetJointAngle(j.name);
        if (j.type == JointType::Revolute || j.type == JointType::Continuous) {
            T = T_jointFrame * glm::rotate(glm::mat4(1.0f), q, j.axis);
        } else {
            T = T_jointFrame;
        }
    }

    FKResult out;
    out.pos = glm::vec3(T * glm::vec4(0,0,0,1));
    out.rot = glm::normalize(glm::quat_cast(glm::mat3(T)));
    return out;
}

static float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

static float wrapPi(float a) {
    while (a >  glm::pi<float>()) a -= 2.0f * glm::pi<float>();
    while (a < -glm::pi<float>()) a += 2.0f * glm::pi<float>();
    return a;
}

// -------- Pose CCD IK: position + orientation --------
//
// Idea per joint:
//  - position correction: same CCD “point to target” angle about joint axis
//  - orientation correction: take q_err = q_target * inverse(q_current)
//      represent as axis u and angle ang, then joint can only rotate about axisW:
//      delta_orient = ang * dot(axisW, u)
//  - total delta = delta_pos + rotWeight * delta_orient
//
static bool solveIK_CCD_Pose(
    URDFRobot& urdf,
    const ChainInfo& chain,
    const glm::vec3& targetPosWorld,
    const glm::quat& targetRotWorld,
    int maxIterations,
    float posTolerance,
    float rotToleranceRad,
    float maxStepDeg,
    float rotWeight
) {
    if (chain.jointIdx.empty()) return false;

    const float maxStepRad = glm::radians(maxStepDeg);
    bool converged = false;

    std::vector<glm::mat4> jointFrames;

    for (int it = 0; it < maxIterations; ++it) {
        FKResult fk = computeFK(urdf, chain, jointFrames);

        float posErr = glm::length(targetPosWorld - fk.pos);

        // orientation error magnitude (angle)
        glm::quat qerr = glm::normalize(targetRotWorld * glm::inverse(fk.rot));
        if (qerr.w < 0.0f) qerr = -qerr; // shortest
        float angErr = 2.0f * std::acos(clampf(qerr.w, -1.0f, 1.0f)); // [0..pi]

        if (posErr <= posTolerance && angErr <= rotToleranceRad) {
            converged = true;
            break;
        }

        for (int k = (int)chain.jointIdx.size() - 1; k >= 0; --k) {
            int jIdx = chain.jointIdx[k];
            const URDFJoint& j = urdf.Joints()[jIdx];
            if (!(j.type == JointType::Revolute || j.type == JointType::Continuous)) continue;

            // recompute each joint update
            fk = computeFK(urdf, chain, jointFrames);

            glm::vec3 pEE = fk.pos;
            glm::quat qEE = fk.rot;

            glm::vec3 pJ = glm::vec3(jointFrames[k] * glm::vec4(0,0,0,1));
            glm::vec3 axisW = glm::normalize(glm::vec3(jointFrames[k] * glm::vec4(j.axis, 0.0f)));
            if (glm::length(axisW) < 1e-6f) continue;

            // ---- position CCD delta ----
            float deltaPos = 0.0f;
            {
                glm::vec3 vCur = pEE - pJ;
                glm::vec3 vTgt = targetPosWorld - pJ;

                float lenCur = glm::length(vCur);
                float lenTgt = glm::length(vTgt);

                if (lenCur > 1e-6f && lenTgt > 1e-6f) {
                    vCur /= lenCur;
                    vTgt /= lenTgt;

                    float cosang = clampf(glm::dot(vCur, vTgt), -1.0f, 1.0f);
                    float ang = std::acos(cosang);

                    glm::vec3 c = glm::cross(vCur, vTgt);
                    float s = glm::dot(axisW, c);
                    if (s < 0.0f) ang = -ang;

                    deltaPos = ang;
                }
            }

            // ---- orientation delta (project quaternion error axis onto joint axis) ----
            float deltaRot = 0.0f;
            {
                glm::quat qerrLocal = glm::normalize(targetRotWorld * glm::inverse(qEE));
                if (qerrLocal.w < 0.0f) qerrLocal = -qerrLocal;

                float angle = 2.0f * std::acos(clampf(qerrLocal.w, -1.0f, 1.0f));
                angle = wrapPi(angle);

                float s = std::sqrt(std::max(0.0f, 1.0f - qerrLocal.w * qerrLocal.w));
                glm::vec3 u(0,0,0);
                if (s > 1e-6f) {
                    u = glm::vec3(qerrLocal.x, qerrLocal.y, qerrLocal.z) / s; // world axis
                }

                // joint can only rotate about axisW
                deltaRot = angle * glm::dot(axisW, u);
            }

            float delta = deltaPos + rotWeight * deltaRot;

            // per-step clamp
            delta = clampf(delta, -maxStepRad, maxStepRad);

            float q = urdf.GetJointAngle(j.name);
            q += delta;

            if (j.type == JointType::Revolute && j.hasLimits) {
                q = clampf(q, j.lower, j.upper);
            }

            urdf.SetJointAngle(j.name, q);

            // early break if we’re good
            fk = computeFK(urdf, chain, jointFrames);

            float pe = glm::length(targetPosWorld - fk.pos);
            glm::quat qe = glm::normalize(targetRotWorld * glm::inverse(fk.rot));
            if (qe.w < 0.0f) qe = -qe;
            float re = 2.0f * std::acos(clampf(qe.w, -1.0f, 1.0f));

            if (pe <= posTolerance && re <= rotToleranceRad) {
                converged = true;
                break;
            }
        }

        if (converged) break;
    }

    return converged;
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

    RobotScene robot;
    bool robotLoaded = false;
    std::string robotErr;

    std::string urdfPath   = pickExistingPath("robot/your_robot.urdf", "build/robot/your_robot.urdf");
    std::string meshesRoot = pickExistingPath("robot/meshes",         "build/robot/meshes");

    ChainInfo chain;
    bool chainBuilt = false;

    auto loadRobot = [&]() {
        try {
            robotLoaded = robot.LoadURDF(urdfPath, meshesRoot);
            robotErr.clear();
            if (!robotLoaded) {
                robotErr = "RobotScene::LoadURDF returned false (paths or parse failed).";
                chainBuilt = false;
                return;
            }
            chain = buildSerialChain(robot.Robot());
            chainBuilt = !chain.jointIdx.empty();
            if (!chainBuilt) robotErr = "Loaded robot, but failed to build a serial joint chain.";
        } catch (const std::exception& e) {
            robotLoaded = false;
            robotErr = e.what();
            chainBuilt = false;
        }
    };

    loadRobot();

    // Gizmo
    static ImGuizmoLite::Gizmo gizmo;
    static bool gizmoInit = false;

    gizmo.s.baseAxisLen        = 0.30f;
    gizmo.s.baseAxisRadius     = 0.020f;
    gizmo.s.baseRingTubeRadius = 0.018f;

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

    static int   ikMaxIter = 40;
    static float ikPosTol = 0.002f;
    static float ikRotTol = 0.02f;     // radians (~1.1 deg)
    static float ikMaxStepDeg = 6.0f;
    static float ikRotWeight = 0.8f;

    while (!glfwWindowShouldClose(window_)) {
        beginFrame();
        ImGuiIO& io = ImGui::GetIO();

        bool rDown = glfwGetKey(window_, GLFW_KEY_R) == GLFW_PRESS;
        if (rDown && !rWasDown) camera.resetTarget();
        rWasDown = rDown;

        double mx, my;
        glfwGetCursorPos(window_, &mx, &my);

        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);

        bool lmbDown = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        bool lmbPressed  = (lmbDown && !prevMouseDown);
        bool lmbReleased = (!lmbDown && prevMouseDown);
        prevMouseDown = lmbDown;

        // init gizmo once
        if (!gizmoInit) {
            ImGuizmoLite::Pose p;
            p.pos = glm::vec3(0.0f);
            p.rot = glm::quat(1,0,0,0);
            gizmo.setTarget(p);
            gizmoInit = true;
        }

        // Update gizmo
        ImGuizmoLite::Pose gizmoPose = gizmo.target;

        glm::vec3 camPos = camera.getCamPos();
        glm::vec3 camForward = glm::normalize(gizmoPose.pos - camPos);

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

        if (!gizmo.capturingMouse) {
            camera.updateFromInput(io.WantCaptureMouse);
        }

        glm::mat4 view = camera.view();
        glm::mat4 proj = camera.orthoProjFromRadius(window_);

        shader.use();
        shader.setMat4("uView", view);
        shader.setMat4("uProj", proj);
        shader.setBool("uUseUniformColor", false);
        shader.setFloat("uAlpha", 1.0f);

        // ---- UI ----
        ImGui::Begin("ArmViz");

        ImGui::Text("URDF: %s", urdfPath.c_str());
        ImGui::Text("Meshes: %s", meshesRoot.c_str());

        if (!robotLoaded) {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1,0.35f,0.35f,1), "Robot NOT loaded");
            ImGui::TextWrapped("%s", robotErr.c_str());
            if (ImGui::Button("Retry Load")) loadRobot();
        } else {
            if (ImGui::Button("Reload URDF")) loadRobot();

            ImGui::Separator();
            ImGui::Text("IK (CCD Pose)");
            ImGui::Checkbox("Enable IK", &ikEnabled);
            ImGui::SameLine();
            ImGui::Checkbox("Solve every frame", &ikSolveEveryFrame);
            ImGui::Checkbox("Use orientation", &ikUseOrientation);

            ImGui::SliderInt("Max iters", &ikMaxIter, 1, 200);
            ImGui::SliderFloat("Pos tol (m)", &ikPosTol, 0.0005f, 0.02f, "%.4f");
            ImGui::SliderFloat("Rot tol (rad)", &ikRotTol, 0.002f, 0.2f, "%.3f");
            ImGui::SliderFloat("Max step (deg)", &ikMaxStepDeg, 0.5f, 15.0f, "%.1f");
            ImGui::SliderFloat("Rot weight", &ikRotWeight, 0.0f, 2.0f, "%.2f");

            if (chainBuilt) {
                if (ImGui::Button("Snap gizmo to EE (pos+rot)")) {
                    std::vector<glm::mat4> jf;
                    FKResult fk = computeFK(robot.Robot(), chain, jf);
                    gizmo.target.pos = fk.pos;
                    gizmo.target.rot = fk.rot;
                }
            } else {
                ImGui::TextColored(ImVec4(1,0.6f,0.2f,1), "Chain not built: %s", robotErr.c_str());
            }

            ImGui::Separator();
            ImGui::Text("Joints (radians):");

            auto& urdf = robot.Robot();
            for (const auto& j : urdf.Joints()) {
                if (j.type == JointType::Fixed) continue;

                float q = urdf.GetJointAngle(j.name);
                float lo = j.hasLimits ? j.lower : -3.14159f;
                float hi = j.hasLimits ? j.upper :  3.14159f;

                if (ImGui::SliderFloat(j.name.c_str(), &q, lo, hi)) {
                    urdf.SetJointAngle(j.name, q);
                }
            }
        }

        ImGui::Separator();
        ImGui::Text("Target:");
        ImGui::Text("pos: %.3f %.3f %.3f", gizmo.target.pos.x, gizmo.target.pos.y, gizmo.target.pos.z);
        ImGui::End();

        // ---- IK solve ----
        if (robotLoaded && chainBuilt && ikEnabled) {
            bool shouldSolve = ikSolveEveryFrame || gizmo.capturingMouse;
            if (shouldSolve) {
                if (ikUseOrientation) {
                    (void)solveIK_CCD_Pose(
                        robot.Robot(),
                        chain,
                        gizmo.target.pos,
                        gizmo.target.rot,
                        ikMaxIter,
                        ikPosTol,
                        ikRotTol,
                        ikMaxStepDeg,
                        ikRotWeight
                    );
                } else {
                    // position-only fallback: pose solver with rotWeight=0
                    (void)solveIK_CCD_Pose(
                        robot.Robot(),
                        chain,
                        gizmo.target.pos,
                        gizmo.target.rot,
                        ikMaxIter,
                        ikPosTol,
                        999.0f,
                        ikMaxStepDeg,
                        0.0f
                    );
                }
            }
        }

        // ---- Draw grid ----
        shader.setMat4("uModel", glm::mat4(1.0f));
        grid.drawLines();

        // ---- Draw robot ----
        if (robotLoaded) robot.Draw(shader);

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
