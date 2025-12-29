#include "RobotScene.h"
#include "../Renderer/Shader.h"
#include "../Renderer/Mesh.h"
#include "../Linker/origin.h"
#include "../Linker/linkJoint.h"
#include "../InverseKinematics/IK.h"
#include <memory>


#include <fstream>
#include <iostream>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <imgui.h>
#include <glm/gtc/matrix_transform.hpp>

RobotScene::RobotScene(const std::string& jsonPath) {
    loadFromJson(jsonPath);

    robot_ = std::make_unique<linkJoint>(origins_.at(0));
    for (size_t i = 1; i < origins_.size(); ++i) {
        robot_->addJoint(origins_.at(i));
    }

    end_ = &origins_.back();

    robot_->setLinkRadius(0.05f);
    robot_->setLinkSlices(18);

    ik_ = std::make_unique<IK>(*robot_);
}


void RobotScene::loadFromJson(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) throw std::runtime_error("Failed to open " + path);

    nlohmann::json j;
    file >> j;

    const auto& arr = j.at("origins");
    origins_.clear();
    origins_.reserve(arr.size());

    for (const auto& o : arr) {
        glm::vec3 pos(
            o.at("position").at(0).get<float>(),
            o.at("position").at(1).get<float>(),
            o.at("position").at(2).get<float>()
        );

        int xDir = o.at("xDir").get<int>();
        int zDir = o.at("zDir").get<int>();

        float axisLength = o.value("axisLength", 0.2f);
        int rotateAxis   = o.value("rotateAxis", 3);
        float angleDeg   = o.value("angleDeg", 0.0f);

        float minDeg = o.value("minDeg", -180.0f);
        float maxDeg = o.value("maxDeg",  180.0f);

        if (o.contains("limits")) {
            const auto& lim = o.at("limits");
            minDeg = lim.value("minDeg", minDeg);
            maxDeg = lim.value("maxDeg", maxDeg);
        }
        bool enableLimits = o.contains("limits");

        origins_.emplace_back(
            pos, xDir, zDir,
            axisLength,
            rotateAxis,
            angleDeg,
            0.02f,
            16,
            minDeg,
            maxDeg,
            enableLimits
        );
    }

    // your “pretty defaults”
    for (auto& org : origins_) {
        org.setAxisRadius(0.015f);
        org.setAxisLength(0.25f);
        org.setCylinderSlices(20);
    }

    std::cout << "origins loaded = " << origins_.size() << "\n";
}

void RobotScene::reset() {
    robot_->reset();
}

void RobotScene::uiAndSolve() {
    ImGui::Begin("IK Controls");
    ImGui::Checkbox("Enable IK", &ikEnabled_);
    ImGui::SliderInt("Iterations / frame", &itersPerFrame_, 1, 30);
    ImGui::SliderFloat("Damping (lambda)", &lambda_, 0.01f, 1.0f);
    ImGui::SliderFloat("Max step (deg)", &maxStepDeg_, 0.1f, 10.0f);
    ImGui::DragFloat3("Target Pos", &targetWorld_.x, 0.01f);
    ImGui::DragFloat3("Target RPY (deg)", &targetEulerDeg_.x, 1.0f, -180.0f, 180.0f);
    ImGui::SliderFloat("Rot Weight", &rotWeight_, 0.0f, 3.0f);

    if (ImGui::Button("Reset Robot (K)")) reset();
    ImGui::SameLine();
    if (ImGui::Button("Target = End Effector") && end_) targetWorld_ = end_->getPos();

    if (end_) {
        ImGui::Text("EE: (%.3f, %.3f, %.3f)", end_->getPos().x, end_->getPos().y, end_->getPos().z);
    }
    ImGui::End();

    if (!ikEnabled_) return;

    glm::vec3 eulerRad = glm::radians(targetEulerDeg_);
    glm::quat targetRot = glm::quat(eulerRad);

    ik_->solvePose(
        targetWorld_,
        targetRot,
        -1,
        itersPerFrame_,
        1e-3f,
        1e-2f,
        lambda_,
        maxStepDeg_,
        rotWeight_
    );
}

void RobotScene::draw(const Shader& shader, const Mesh& sphereWire) {
    // Update link geometry buffers (you currently rebuild every frame)
    robot_->verts.clear();
    robot_->addVerts();
    robot_->setupBuffers();

    // Draw target sphere (wireframe)
    glDepthMask(GL_FALSE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glm::mat4 m(1.0f);
    m = glm::translate(m, targetWorld_);
    shader.setMat4("uModel", m);
    sphereWire.drawTriangles();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDepthMask(GL_TRUE);

    // Links translucent
    glDepthMask(GL_FALSE);
    shader.setFloat("uAlpha", 0.35f);
    robot_->link(shader.id(), glGetUniformLocation(shader.id(), "uModel"));
    glDepthMask(GL_TRUE);

    // Origins solid
    shader.setFloat("uAlpha", 1.0f);
    for (const Origin& o : origins_) {
        o.draw(shader.id(), glGetUniformLocation(shader.id(), "uModel"));
    }
}
