
#pragma once
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
#include "Trajectory/Jog.h"
#include "Utilities/Utilities.h"
#include "Objects/Cube.h"

#ifndef PROJECT_ROOT_DIR
#define PROJECT_ROOT_DIR "."
#endif

static void glfw_error_callback(int error, const char* description)
{
    (void)error;
    (void)description;
}

App::App()
{
    glfwSetErrorCallback(glfw_error_callback);
    initGLFW();
    createWindow();
    initGLAD();
    initOpenGLState();
    initImGui();
}

App::~App()
{
    shutdownImGui();
    if (window_)
    {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}

void App::initGLFW()
{
    if (!glfwInit()) throw std::runtime_error("GLFW init failed");

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
}

void App::createWindow()
{
    window_ = glfwCreateWindow(1280, 720, "ArmViz", nullptr, nullptr);
    if (!window_) throw std::runtime_error("Window creation failed");

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
}

void App::initGLAD()
{
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        throw std::runtime_error("GLAD init failed");
    }
}

void App::initOpenGLState()
{
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void App::initImGui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void App::shutdownImGui()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void App::beginFrame()
{
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

void App::endFrame()
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window_);
}

static std::string pickExistingPath(const std::string& a, const std::string& b)
{
    if (std::filesystem::exists(a)) return a;
    if (std::filesystem::exists(b)) return b;
    return a;
}




void App::run()
{
    Utilities utils;

    Shader shader(utils.vsSrc, utils.fsSrc);

    Mesh grid = Mesh::fromVertexColorLines(makeGridVerts(10));
    Mesh unitCyl = Mesh::fromVertexColorTriangles(makeCylinderVerts(1.0f, 1.0f, 24));

    OrbitCamera camera;
    camera.attachToWindow(window_);

    RobotScene robotA;
    RobotScene robotB;

    bool robotLoadedA = false;
    bool robotLoadedB = false;
    std::string robotErrA;
    std::string robotErrB;

    std::string urdfPath = std::string(PROJECT_ROOT_DIR) + "/robotModel/your_robot.urdf";
    std::string meshesRoot = std::string(PROJECT_ROOT_DIR) + "/robotModel/meshes";

    // IMPORTANT: chain must be built PER-robot instance
    URDFIK::ChainInfo chainA;
    URDFIK::ChainInfo chainB;
    bool chainBuiltA = false;
    bool chainBuiltB = false;


    utils.loadRobots(
        &robotLoadedA,
        &robotLoadedB,
        &chainBuiltA,
        &chainBuiltB,
        &robotErrA,
        &robotErrB,
        &chainA,
        &chainB,
        &robotA,
        &robotB,
        urdfPath,
        meshesRoot
    );

    // Gizmo
    static ImGuizmoLite::Gizmo gizmo;
    static bool gizmoInit = false;


    bool rWasDown = false;
    static bool prevMouseDown = false;

    // IK controls
    static bool ikEnabled = true;
    static bool ikSolveEveryFrame = true;
    static bool ikUseOrientation = true;

    static int ikMaxIter = 60;
    static float ikPosTol = 0.002f;
    static float ikRotTol = 0.02f;
    static float ikMaxStepDeg = 6.0f;
    static float ikRotWeight = 1.0f;
    static float ikLambda = 0.20f;

    // Jog controls
    static int density = 1000;
    static bool jogging = false;
    static Trajectory traj;
    static Trajectory trajTempRender;
    static Jog JogTempRender;
    static Jog jogger;

    static int jogStride = 1;
    static bool jogInterpRotation = true;

    static glm::vec3 jogEndPos(0.0f);
    static glm::quat jogEndRot(1, 0, 0, 0);


    static bool renderTrajectory = false;
    static URDFIK::FKResult renderTrajectoryPos;
    static bool renderingTrajectory = false;
    static float cubeScale = 0.05f;
    static Cube cube(glm::vec3(0.0f, 0.5f, cubeScale/2.0f), glm::quat(1,0,0,0), glm::vec3(cubeScale));
    cube.SetParent(&robotB, &chainA);

    while (!glfwWindowShouldClose(window_))
    {
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

        float mx = (float)mxWin * sx;
        float my = (float)myWin * sy;

        bool lmbDown = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        bool lmbPressed = (lmbDown && !prevMouseDown);
        bool lmbReleased = (!lmbDown && prevMouseDown);
        prevMouseDown = lmbDown;

        // init gizmo once (snap to robotA EE)
        if (!gizmoInit && robotLoadedA && chainBuiltA)
        {
            std::vector<glm::mat4> jf;
            URDFIK::FKResult fk = URDFIK::ComputeFK(robotA.Robot(), chainA, jf);
            gizmo.target.pos = fk.pos;
            gizmo.target.rot = fk.rot;
            gizmoInit = true;
        }

        if (!gizmo.capturingMouse)
        {
            camera.updateFromInput(io.WantCaptureMouse);
        }

        glm::mat4 view = camera.view();
        glm::mat4 proj = camera.orthoProjFromRadius(window_);
        glm::vec3 camPos = camera.getCamPos();
        glm::vec3 camForward = glm::normalize(-glm::vec3(view[2]));
        if (!renderTrajectory)
        {
            gizmo.update(
                gizmo.target,
                view,
                proj,
                camPos,
                camForward,
                mx, my,
                (float)fbW, (float)fbH,
                lmbDown, lmbPressed, lmbReleased
            );
        }
        gizmo.target.rot = glm::normalize(gizmo.target.rot);
        shader.use();
        shader.setMat4("uView", view);
        shader.setMat4("uProj", proj);

        // defaults
        shader.setBool("uUseUniformColor", false);
        shader.setFloat("uAlpha", 1.0f);
        shader.setBool("uTintEnabled", false);
        shader.setVec3("uTintColor", glm::vec3(1.0f, 1.0f, 1.0f));
        shader.setFloat("uTintStrength", 0.0f);

        // ---- UI ----
        ImGui::Begin("ArmViz");

        ImGui::Text("URDF: %s", urdfPath.c_str());
        ImGui::Text("Meshes: %s", meshesRoot.c_str());

        if (!robotLoadedA || !robotLoadedB)
        {
            ImGui::Separator();
            if (!robotLoadedA)
            {
                ImGui::TextColored(ImVec4(1, 0.35f, 0.35f, 1), "robotA NOT loaded");
                ImGui::TextWrapped("%s", robotErrA.c_str());
            }
            if (!robotLoadedB)
            {
                ImGui::TextColored(ImVec4(1, 0.35f, 0.35f, 1), "robotB NOT loaded");
                ImGui::TextWrapped("%s", robotErrB.c_str());
            }
            if (ImGui::Button("Retry Load"))
                utils.loadRobots(&robotLoadedA,
                           &robotLoadedB,
                           &chainBuiltA,
                           &chainBuiltB,
                           &robotErrA,
                           &robotErrB,
                           &chainA,
                           &chainB,
                           &robotA,
                           &robotB,
                           urdfPath,
                           meshesRoot);
        }
        else
        {
            if (ImGui::Button("Reload URDF (both)"))
            {
                gizmoInit = false;
                utils.loadRobots(&robotLoadedA,
                           &robotLoadedB,
                           &chainBuiltA,
                           &chainBuiltB,
                           &robotErrA,
                           &robotErrB,
                           &chainA,
                           &chainB,
                           &robotA,
                           &robotB,
                           urdfPath,
                           meshesRoot);
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
            ImGui::Checkbox("Render Trajectory", &renderTrajectory);
            ImGui::Text("Jog:");
            ImGui::SliderInt("Jog Density", &density, 100, 5000);
            ImGui::SliderInt("Jog Stride (pts/frame)", &jogStride, 1, 50);
            ImGui::Checkbox("Interpolate Rotation (slerp)", &jogInterpRotation);
            if (ImGui::Button("JOG ROBOT") && !jogging)
            {
                if (robotLoadedB && chainBuiltB)
                {
                    jogging = true;

                    jogEndPos = gizmo.target.pos;
                    jogEndRot = gizmo.target.rot;

                    std::vector<glm::mat4> jfB;
                    URDFIK::FKResult fkB = URDFIK::ComputeFK(robotB.Robot(), chainB, jfB);

                    if (jogInterpRotation)
                    {
                        traj.GeneratePoses(fkB.pos, fkB.rot, jogEndPos, jogEndRot, density);
                    }
                    else
                    {
                        traj.GeneratePoints(fkB.pos, jogEndPos, density);
                    }
                }
            }

            if (chainBuiltA)
            {
                if (ImGui::Button("Snap gizmo to robotA EE (pos+rot)"))
                {
                    std::vector<glm::mat4> jf;
                    URDFIK::FKResult fk = URDFIK::ComputeFK(robotA.Robot(), chainA, jf);
                    gizmo.target.pos = fk.pos;
                    gizmo.target.rot = fk.rot;
                }
            }

            ImGui::Separator();
            if (ImGui::Button("Toggle Grab"))
                cube.SetGrabbed(!cube.grabbed);
            ImGui::Separator();
            ImGui::Text("Joints (radians) [editing robotA]:");


            auto& urdfA = robotA.Robot();
            for (const auto& j : urdfA.Joints())
            {
                if (j.type == JointType::Fixed) continue;

                float q = urdfA.GetJointAngle(j.name);
                float lo = j.hasLimits ? j.lower : -3.14159f;
                float hi = j.hasLimits ? j.upper : 3.14159f;

                if (ImGui::SliderFloat(j.name.c_str(), &q, lo, hi))
                {
                    urdfA.SetJointAngle(j.name, q);
                }
            }
        }

        ImGui::Separator();
        ImGui::Text("Target:");
        ImGui::Text("pos: %.3f %.3f %.3f", gizmo.target.pos.x, gizmo.target.pos.y, gizmo.target.pos.z);
        ImGui::End();

        // ---- Render Trajectory or Pos ----
        if (renderTrajectory && !renderingTrajectory)
        {
            renderingTrajectory = true;

            std::vector<glm::mat4> jf;
            URDFIK::FKResult fk = URDFIK::ComputeFK(robotB.Robot(), chainB, jf);

            renderTrajectoryPos.pos = gizmo.target.pos;
            renderTrajectoryPos.rot = gizmo.target.rot;

            if (jogInterpRotation) trajTempRender.GeneratePoses(fk.pos, fk.rot, renderTrajectoryPos.pos,
                                                                renderTrajectoryPos.rot, density);
            else trajTempRender.GeneratePoints(fk.pos, renderTrajectoryPos.pos, density);
        }
        if (!renderTrajectory && renderingTrajectory)
        {
            renderingTrajectory = false;
            JogTempRender.stopJogging();
        }

        if (renderingTrajectory && robotLoadedA && chainBuiltA && ikEnabled)
        {
            JogTempRender.startJog(jogInterpRotation,
                                   &trajTempRender, &robotA,
                                   ikUseOrientation,
                                   ikPosTol,
                                   ikRotTol,
                                   ikRotWeight,
                                   jogStride,
                                   ikMaxStepDeg,
                                   ikLambda,
                                   ikMaxIter,
                                   &chainA,
                                   jogEndRot);
            if (!JogTempRender.getJoggingStatus()) JogTempRender.restartJogging();
        }

        else if (robotLoadedA && chainBuiltA && ikEnabled)
        {
            if (ikSolveEveryFrame || gizmo.capturingMouse)
            {
                float rotTolUse = ikUseOrientation ? ikRotTol : 999.0f;
                float rotWUse = ikUseOrientation ? ikRotWeight : 0.0f;

                (void)URDFIK::SolvePoseHierDLS(
                    robotA.Robot(),
                    chainA,
                    gizmo.target.pos,
                    gizmo.target.rot,
                    ikMaxIter,
                    ikPosTol,
                    rotTolUse,
                    ikMaxStepDeg,
                    rotWUse,
                    ikLambda
                );
            }
        }

        // ---- Jog solve (robotB only) ----
        if (jogging)
        {
            renderTrajectory = false;
            if (!robotLoadedB || !chainBuiltB || !ikEnabled)
            {
                jogging = false;
            }
            else
            {
                jogger.startJog(jogInterpRotation,
                                &traj, &robotB,
                                ikUseOrientation,
                                ikPosTol,
                                ikRotTol,
                                ikRotWeight,
                                jogStride,
                                ikMaxStepDeg,
                                ikLambda,
                                ikMaxIter,
                                &chainB,
                                jogEndRot);
                jogging = jogger.getJoggingStatus();
            }
        }


        cube.UpdateFromParentEE();

        // draw cube (solid)
        shader.setBool("uUseUniformColor", true);
        shader.setVec3("uUniformColor", glm::vec3(0.2f, 0.8f, 1.0f));
        shader.setFloat("uAlpha", 1.0f);
        cube.Draw(shader);

        // restore if you want (optional)
        shader.setBool("uUseUniformColor", false);
        shader.setFloat("uAlpha", 1.0f);


        // ---- Draw grid ----
        shader.setBool("uUseUniformColor", false);
        shader.setBool("uTintEnabled", false);
        shader.setFloat("uAlpha", 1.0f);
        shader.setMat4("uModel", glm::mat4(1.0f));
        grid.drawLines();

        // ---- Draw robotB normally (real robot) ----
        shader.setBool("uUseUniformColor", false);
        shader.setBool("uTintEnabled", false);
        shader.setFloat("uAlpha", 1.0f);
        if (robotLoadedB) robotB.Draw(shader);

        shader.setBool("uUseUniformColor", true);
        shader.setVec3("uUniformColor", glm::vec3(1.0f, 0.569f, 0.0f));
        shader.setMat4("uModel", glm::mat4(1.0f));
        shader.setFloat("uAlpha", 0.5f); // Opacity
        if (robotLoadedA) robotA.Draw(shader);


        // ---- Draw gizmo at target ----

        utils.gizmoCyls.clear();
        gizmo.draw(gizmo.target, proj, utils.drawCylinderCallback());

        shader.setBool("uTintEnabled", false);
        shader.setBool("uUseUniformColor", true);
        shader.setFloat("uAlpha", 1.0f);

        for (const auto& seg : utils.gizmoCyls)
        {
            shader.setVec3("uUniformColor", seg.c);
            shader.setMat4("uModel", utils.cylinderModel(seg.a, seg.b, seg.r));
            unitCyl.drawTriangles();
        }

        shader.setBool("uUseUniformColor", false);
        shader.setFloat("uAlpha", 1.0f);

        endFrame();
    }
}
