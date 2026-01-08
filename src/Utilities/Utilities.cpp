#define GLM_ENABLE_EXPERIMENTAL

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <iostream>
#include <stdexcept>
#include <glad/glad.h>
#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>
#include <cmath>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "../Renderer/Shader.h"
#include "../Renderer/Mesh.h"
#include "../Renderer/Primitives.h"
#include "../Camera/OrbitCamera.h"
#include "../Robot/RobotScene.h"
#include "../Robot/URDFRobot.h"
#include "../ImGuizmo/ImGuizmo.h"
#include "../InverseKinematics/IK.h"
#include "../Trajectory/Trajectory.h"
#include "Utilities.h"
#include <functional>



void Utilities::loadRobots(
    bool* robotLoadedA,
    bool* robotLoadedB,
    bool* chainBuiltA,
    bool* chainBuiltB,
    std::string* robotErrA,
    std::string* robotErrB,
    URDFIK::ChainInfo* chainA,
    URDFIK::ChainInfo* chainB,
    RobotScene* robotA,
    RobotScene* robotB,
    std::string urdfPath,
    std::string meshesRoot
)
{
    *robotLoadedA = *robotLoadedB = false;
    robotErrA->clear();
    robotErrB->clear();
    *chainBuiltA = *chainBuiltB = false;
    *chainA = URDFIK::ChainInfo{};
    *chainB = URDFIK::ChainInfo{};

    // robotA
    try
    {
        *robotLoadedA = robotA->LoadURDF(urdfPath, meshesRoot);
        if (!*robotLoadedA)
        {
            *robotErrA = "RobotScene::LoadURDF returned false (paths or parse failed).";
        }
        else
        {
            *chainA = URDFIK::BuildSerialChain(robotA->Robot());
            *chainBuiltA = !chainA->jointIdx.empty();
            if (!*chainBuiltA) *robotErrA = "Loaded robotA, but failed to build a serial joint chain.";
        }
    }
    catch (const std::exception& e)
    {
        *robotLoadedA = false;
        *robotErrA = e.what();
        *chainBuiltA = false;
    }

    // robotB
    try
    {
        *robotLoadedB = robotB->LoadURDF(urdfPath, meshesRoot);
        if (!*robotLoadedB)
        {
            *robotErrB = "RobotScene::LoadURDF returned false (paths or parse failed).";
        }
        else
        {
            *chainB = URDFIK::BuildSerialChain(robotB->Robot());
            *chainBuiltB = !chainB->jointIdx.empty();
            if (!*chainBuiltB) *robotErrB = "Loaded robotB, but failed to build a serial joint chain.";
        }
    }
    catch (const std::exception& e)
    {
        *robotLoadedB = false;
        *robotErrB = e.what();
        *chainBuiltB = false;
    }
};
std::function<void(const glm::vec3&, const glm::vec3&, float, const glm::vec3&)>
Utilities::drawCylinderCallback()
{
    return [this](const glm::vec3& a, const glm::vec3& b, float r, const glm::vec3& c)
    {
        this->drawCylinder(a, b, r, c); // pushes into gizmoCyls
    };
}
void Utilities::drawCylinder(const glm::vec3& a, const glm::vec3& b, float radius, const glm::vec3& color)
{
    gizmoCyls.push_back({a, b, radius, color});
};

auto Utilities::cylinderModel(const glm::vec3& a, const glm::vec3& b, float radius) -> glm::mat4
{
    glm::vec3 d = b - a;
    float len = glm::length(d);
    if (len < 1e-6f) return glm::mat4(1.0f);

    glm::vec3 z = d / len;
    glm::vec3 up = (std::fabs(z.y) < 0.99f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
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

