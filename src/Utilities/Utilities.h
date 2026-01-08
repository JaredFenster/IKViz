//
// Created by jared on 2026-01-07.
//
#define GLM_ENABLE_EXPERIMENTAL
#ifndef ARMVIZ_UTILITIES_H
#define ARMVIZ_UTILITIES_H
#endif //ARMVIZ_UTILITIES_H

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
#include "../Trajectory/Jog.h"

class Utilities
{
public:

    struct GizmoCyl
    {
        glm::vec3 a, b;
        float r;
        glm::vec3 c;
    };

    void loadRobots(
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
    );

    void drawCylinder(const glm::vec3& a, const glm::vec3& b, float radius, const glm::vec3& color);
    std::function<void(const glm::vec3&, const glm::vec3&, float, const glm::vec3&)>
    drawCylinderCallback();

    auto cylinderModel(const glm::vec3& a, const glm::vec3& b, float radius) -> glm::mat4;

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

        // tint mode (mixes tint with existing vColor/uniform)
        uniform bool  uTintEnabled;
        uniform vec3  uTintColor;
        uniform float uTintStrength; // 0 = no tint, 1 = full tint

        void main() {
            vec3 col = uUseUniformColor ? uUniformColor : vColor;

            if (uTintEnabled) {
                col = mix(col, uTintColor, clamp(uTintStrength, 0.0, 1.0));
            }

            FragColor = vec4(col, uAlpha);
        }
    )GLSL";

    std::vector<GizmoCyl> gizmoCyls;
};