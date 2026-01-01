#include "App.h"
#define GLM_ENABLE_EXPERIMENTAL

#include <stdexcept>
#include <glad/glad.h>
#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>
#include <cmath>
#include <array>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/constants.hpp>

#include "Renderer/Shader.h"
#include "Renderer/Mesh.h"
#include "Renderer/Primitives.h"
#include "Camera/OrbitCamera.h"
#include "Robot/RobotScene.h"
#include "Robot/URDFRobot.h"
#include "Robot/URDFMath.h"
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
    std::vector<int> jointIdx;
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
        int jIdx = childs[0];
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

// --------- Small linear algebra (3x3) ---------

static bool solve3x3(float A[3][3], float b[3]) {
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        float best = std::fabs(A[pivot][col]);
        for (int r = col + 1; r < 3; ++r) {
            float v = std::fabs(A[r][col]);
            if (v > best) { best = v; pivot = r; }
        }
        if (best < 1e-10f) return false;

        if (pivot != col) {
            for (int c = 0; c < 3; ++c) std::swap(A[pivot][c], A[col][c]);
            std::swap(b[pivot], b[col]);
        }

        float diag = A[col][col];
        for (int c = 0; c < 3; ++c) A[col][c] /= diag;
        b[col] /= diag;

        for (int r = 0; r < 3; ++r) {
            if (r == col) continue;
            float f = A[r][col];
            if (std::fabs(f) < 1e-12f) continue;
            for (int c = 0; c < 3; ++c) A[r][c] -= f * A[col][c];
            b[r] -= f * b[col];
        }
    }
    return true;
}

static bool invert3x3(const float M[3][3], float invOut[3][3]) {
    for (int i = 0; i < 3; ++i) {
        float A[3][3] = {
            { M[0][0], M[0][1], M[0][2] },
            { M[1][0], M[1][1], M[1][2] },
            { M[2][0], M[2][1], M[2][2] },
        };
        float b[3] = { 0,0,0 };
        b[i] = 1.0f;
        if (!solve3x3(A, b)) return false;
        invOut[0][i] = b[0];
        invOut[1][i] = b[1];
        invOut[2][i] = b[2];
    }
    return true;
}

static bool solveTaskDLS3(
    const std::vector<glm::vec3>& Jcols,
    const glm::vec3& e,
    float lambda,
    std::vector<float>& outDQ
) {
    const int n = (int)Jcols.size();
    outDQ.assign(n, 0.0f);

    float M[3][3] = { {0,0,0},{0,0,0},{0,0,0} };

    for (int i = 0; i < n; ++i) {
        const glm::vec3& Ji = Jcols[i];
        M[0][0] += Ji.x * Ji.x; M[0][1] += Ji.x * Ji.y; M[0][2] += Ji.x * Ji.z;
        M[1][0] += Ji.y * Ji.x; M[1][1] += Ji.y * Ji.y; M[1][2] += Ji.y * Ji.z;
        M[2][0] += Ji.z * Ji.x; M[2][1] += Ji.z * Ji.y; M[2][2] += Ji.z * Ji.z;
    }

    const float lam2 = lambda * lambda;
    M[0][0] += lam2; M[1][1] += lam2; M[2][2] += lam2;

    float A[3][3] = {
        { M[0][0], M[0][1], M[0][2] },
        { M[1][0], M[1][1], M[1][2] },
        { M[2][0], M[2][1], M[2][2] },
    };
    float b[3] = { e.x, e.y, e.z };

    if (!solve3x3(A, b)) return false;

    glm::vec3 y(b[0], b[1], b[2]);
    for (int i = 0; i < n; ++i) outDQ[i] = glm::dot(Jcols[i], y);
    return true;
}

static void snapshotChainAngles(const URDFRobot& urdf, const ChainInfo& chain, std::vector<float>& out) {
    out.resize(chain.jointIdx.size());
    for (size_t k = 0; k < chain.jointIdx.size(); ++k) {
        const URDFJoint& j = urdf.Joints()[chain.jointIdx[k]];
        out[k] = urdf.GetJointAngle(j.name);
    }
}

static void restoreChainAngles(URDFRobot& urdf, const ChainInfo& chain, const std::vector<float>& snap) {
    for (size_t k = 0; k < chain.jointIdx.size(); ++k) {
        const URDFJoint& j = urdf.Joints()[chain.jointIdx[k]];
        urdf.SetJointAngle(j.name, snap[k]);
    }
}

static glm::vec3 quatErrorAxisAngleWorld(const glm::quat& qCurrentWorld,
                                        const glm::quat& qTargetWorld,
                                        float* outAngleMag = nullptr)
{
    glm::quat qErr = glm::normalize(qTargetWorld * glm::inverse(qCurrentWorld));
    if (qErr.w < 0.0f) qErr = -qErr;

    glm::vec3 v(qErr.x, qErr.y, qErr.z);
    float s = glm::length(v);

    float ang = 2.0f * std::atan2(s, clampf(qErr.w, -1.0f, 1.0f));
    if (outAngleMag) *outAngleMag = std::fabs(ang);

    if (s < 1e-8f || std::fabs(ang) < 1e-8f) return glm::vec3(0.0f);
    glm::vec3 axis = v / s;
    return axis * ang;
}

// -------- Best-results Pose IK: hierarchical (pos primary, rot in nullspace) --------
static bool solveIK_HierDLS_Pose(
    URDFRobot& urdf,
    const ChainInfo& chain,
    const glm::vec3& targetPosWorld,
    const glm::quat& targetRotWorld,
    int maxIterations,
    float posTolerance,
    float rotToleranceRad,
    float maxStepDeg,
    float rotWeight,
    float lambda
) {
    if (chain.jointIdx.empty()) return false;

    const float maxStepRad = glm::radians(maxStepDeg);
    const int n = (int)chain.jointIdx.size();

    std::vector<glm::mat4> jointFrames;
    std::vector<glm::vec3> JpCols(n);
    std::vector<glm::vec3> JrCols(n);

    std::vector<float> dqPos(n, 0.0f);
    std::vector<float> dqRot(n, 0.0f);
    std::vector<float> dq(n, 0.0f);

    float lambdaLocal = std::max(1e-6f, lambda);

    for (int it = 0; it < maxIterations; ++it) {
        FKResult fk = computeFK(urdf, chain, jointFrames);

        glm::vec3 ep = targetPosWorld - fk.pos;
        float posErr0 = glm::length(ep);

        float angErr0 = 0.0f;
        glm::vec3 er = quatErrorAxisAngleWorld(fk.rot, targetRotWorld, &angErr0);

        if (posErr0 <= posTolerance && angErr0 <= rotToleranceRad) return true;

        glm::vec3 pEE = fk.pos;
        for (int k = 0; k < n; ++k) {
            const URDFJoint& j = urdf.Joints()[chain.jointIdx[k]];
            glm::vec3 pJ = glm::vec3(jointFrames[k] * glm::vec4(0,0,0,1));
            glm::vec3 axisW = glm::normalize(glm::vec3(jointFrames[k] * glm::vec4(j.axis, 0.0f)));

            if (glm::length(axisW) < 1e-6f) axisW = glm::vec3(0,0,0);

            JpCols[k] = glm::cross(axisW, (pEE - pJ));
            JrCols[k] = axisW;
        }

        if (!solveTaskDLS3(JpCols, ep, lambdaLocal, dqPos)) return false;

        float ramp = 0.0f;
        {
            float far = std::max(posTolerance * 10.0f, posTolerance + 1e-6f);
            float t = (far - posErr0) / (far - posTolerance);
            ramp = clampf(t, 0.0f, 1.0f);
            ramp = ramp * ramp * (3.0f - 2.0f * ramp);
        }

        float rw = std::max(0.0f, rotWeight) * ramp;

        if (rw > 0.0f) {
            glm::vec3 erScaled = rw * er;
            if (!solveTaskDLS3(JrCols, erScaled, lambdaLocal, dqRot)) {
                dqRot.assign(n, 0.0f);
            }
        } else {
            dqRot.assign(n, 0.0f);
        }
        //here
                // --- Combine steps WITHOUT nullspace projection ---
        // Nullspace projection often kills rotation completely near the target (no redundancy).
        for (int i = 0; i < n; ++i) {
            dq[i] = dqPos[i] + dqRot[i];
        }

        // Optional: if you're basically at the position target, prioritize rotation more
        if (posErr0 <= posTolerance * 3.0f && angErr0 > rotToleranceRad) {
            for (int i = 0; i < n; ++i) dq[i] = dqRot[i];
        }
        //here
        float maxAbs = 0.0f;
        for (int i = 0; i < n; ++i) maxAbs = std::max(maxAbs, std::fabs(dq[i]));
        if (maxAbs > maxStepRad && maxAbs > 1e-12f) {
            float s = maxStepRad / maxAbs;
            for (int i = 0; i < n; ++i) dq[i] *= s;
        }

        std::vector<float> snap;
        snapshotChainAngles(urdf, chain, snap);

        float metric0 = posErr0 * posErr0 + (rw * angErr0) * (rw * angErr0);

        bool accepted = false;
        float alpha = 1.0f;

        for (int ls = 0; ls < 6; ++ls) {
            for (int k = 0; k < n; ++k) {
                const URDFJoint& j = urdf.Joints()[chain.jointIdx[k]];
                if (!(j.type == JointType::Revolute || j.type == JointType::Continuous)) continue;

                float q = urdf.GetJointAngle(j.name);
                q += alpha * dq[k];

                if (j.type == JointType::Revolute && j.hasLimits) {
                    q = clampf(q, j.lower, j.upper);
                }
                urdf.SetJointAngle(j.name, q);
            }

            FKResult fk1 = computeFK(urdf, chain, jointFrames);
            float posErr1 = glm::length(targetPosWorld - fk1.pos);

            float angErr1 = 0.0f;
            (void)quatErrorAxisAngleWorld(fk1.rot, targetRotWorld, &angErr1);

            float metric1 = posErr1 * posErr1 + (rw * angErr1) * (rw * angErr1);

            float posSlack = (posErr0 <= posTolerance * 3.0f) ? (posTolerance * 2.0f) : 1e-9f;

            if (metric1 < metric0 && posErr1 <= posErr0 + posSlack) {
                accepted = true;
                break;
            }


            restoreChainAngles(urdf, chain, snap);
            alpha *= 0.5f;
        }

        if (!accepted) {
            lambdaLocal = std::min(lambdaLocal * 2.0f, 50.0f);
        } else {
            lambdaLocal = std::max(lambdaLocal * 0.7f, 1e-6f);
        }
    }

    return false;
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

    static int   ikMaxIter = 60;
    static float ikPosTol = 0.002f;
    static float ikRotTol = 0.02f;
    static float ikMaxStepDeg = 6.0f;
    static float ikRotWeight = 1.0f;
    static float ikLambda = 0.20f;

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

        // init gizmo once
        if (!gizmoInit) {
            std::vector<glm::mat4> jf;
            FKResult fk = computeFK(robot.Robot(), chain, jf);
            gizmo.target.pos = fk.pos;
            gizmo.target.rot = fk.rot;
            gizmoInit = true;
        }

        // Build view/proj for this frame (used for gizmo + render)
        glm::mat4 view = camera.view();
        glm::mat4 proj = camera.orthoProjFromRadius(window_);
        glm::vec3 camPos = camera.getCamPos();

        // --- Update gizmo (THIS is the fix: do NOT copy+writeback) ---
        glm::vec3 camForward = glm::normalize(gizmo.target.pos - camPos);

        gizmo.update(
            gizmo.target,   // ee == target because gizmo is anchored on target
            view,
            proj,
            camPos,
            camForward,
            mx, my,
            (float)fbW, (float)fbH,
            lmbDown, lmbPressed, lmbReleased
        );

        // --- Now update camera only if gizmo did NOT capture the mouse ---
        if (!gizmo.capturingMouse) {
            camera.updateFromInput(io.WantCaptureMouse);
        }

        // Recompute view/proj after camera move (render should match camera)
        view = camera.view();
        proj = camera.orthoProjFromRadius(window_);

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
            ImGui::Text("IK (Hierarchical DLS Pose)");
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
                    (void)solveIK_HierDLS_Pose(
                        robot.Robot(),
                        chain,
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
                    (void)solveIK_HierDLS_Pose(
                        robot.Robot(),
                        chain,
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
