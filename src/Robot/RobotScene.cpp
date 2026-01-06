#define GLM_ENABLE_EXPERIMENTAL

#include "RobotScene.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "../Renderer/Shader.h"
#include "../Renderer/Mesh.h"
#include "../Robot/URDFMath.h"   // for urdfXYZRPY(...)

bool RobotScene::LoadURDF(const std::string& urdfPath, const std::string& meshesRoot) {
    return robot_.LoadFromFile(urdfPath, meshesRoot);
}

void RobotScene::Draw(const Shader& shader) {
    DrawLinkRecursive(robot_.RootLink(), glm::mat4(1.0f), shader);
}

void RobotScene::DrawPreview(const Shader& shader) {
    // Draw only filled triangles (no edges) so we can render them translucent
    std::function<void(const std::string&, const glm::mat4&)> rec;
    rec = [&](const std::string& linkName, const glm::mat4& parentWorld) {
        const auto& links = robot_.Links();
        auto it = links.find(linkName);
        if (it == links.end()) return;

        const URDFLink& link = it->second;

        if (link.hasVisual && link.mesh) {
            glm::mat4 M = parentWorld;
            M = M * urdfXYZRPY(link.visual.xyz, link.visual.rpy);
            M = glm::scale(M, link.visual.scale);

            shader.setMat4("uModel", M);
            // draw triangles only
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(1.0f, 1.0f);
            link.mesh->drawTriangles();
            glDisable(GL_POLYGON_OFFSET_FILL);
        }

        const auto& childJointIndices = robot_.ChildJointsOf(linkName);
        for (int jIdx : childJointIndices) {
            const URDFJoint& j = robot_.Joints()[jIdx];

            glm::mat4 childWorld = parentWorld;
            childWorld = childWorld * urdfXYZRPY(j.originXyz, j.originRpy);
            if (j.type == JointType::Revolute || j.type == JointType::Continuous) {
                float q = robot_.GetJointAngle(j.name);
                childWorld = childWorld * glm::rotate(glm::mat4(1.0f), q, j.axis);
            }

            rec(j.childLink, childWorld);
        }
    };

    rec(robot_.RootLink(), glm::mat4(1.0f));
}

void RobotScene::DrawEdges(const Shader& shader) {
    // Draw only edge lines (opaque) on top of preview
    std::function<void(const std::string&, const glm::mat4&)> rec;
    rec = [&](const std::string& linkName, const glm::mat4& parentWorld) {
        const auto& links = robot_.Links();
        auto it = links.find(linkName);
        if (it == links.end()) return;

        const URDFLink& link = it->second;

        if (link.hasVisual && link.edgeMesh) {
            glm::mat4 M = parentWorld;
            M = M * urdfXYZRPY(link.visual.xyz, link.visual.rpy);
            M = glm::scale(M, link.visual.scale);

            shader.setMat4("uModel", M);
            // draw lines (edgeMesh already encodes edge color)
            link.edgeMesh->drawLines();
        }

        const auto& childJointIndices = robot_.ChildJointsOf(linkName);
        for (int jIdx : childJointIndices) {
            const URDFJoint& j = robot_.Joints()[jIdx];

            glm::mat4 childWorld = parentWorld;
            childWorld = childWorld * urdfXYZRPY(j.originXyz, j.originRpy);
            if (j.type == JointType::Revolute || j.type == JointType::Continuous) {
                float q = robot_.GetJointAngle(j.name);
                childWorld = childWorld * glm::rotate(glm::mat4(1.0f), q, j.axis);
            }

            rec(j.childLink, childWorld);
        }
    };

    rec(robot_.RootLink(), glm::mat4(1.0f));
}

void RobotScene::DrawLinkRecursive(const std::string& linkName,
                                   const glm::mat4& parentWorld,
                                   const Shader& shader) {
    const auto& links = robot_.Links();
    auto it = links.find(linkName);
    if (it == links.end()) return;

    const URDFLink& link = it->second;

    // 1) Draw this link's visual mesh
    if (link.hasVisual && link.mesh) {
        glm::mat4 M = parentWorld;

        // link visual origin (local to link frame)
        M = M * urdfXYZRPY(link.visual.xyz, link.visual.rpy);

        // URDF mesh scale (your files use 0.001)
        M = glm::scale(M, link.visual.scale);

        shader.setMat4("uModel", M); 
        if (link.edgeMesh) {
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(1.0f, 1.0f);
            link.mesh->drawTriangles();
            glDisable(GL_POLYGON_OFFSET_FILL);
            link.edgeMesh->drawLines();
        }

    }

    // 2) Recurse into child joints
    const auto& childJointIndices = robot_.ChildJointsOf(linkName);
    for (int jIdx : childJointIndices) {
        const URDFJoint& j = robot_.Joints()[jIdx];

        glm::mat4 childWorld = parentWorld;

        // joint origin (from parent link frame)
        childWorld = childWorld * urdfXYZRPY(j.originXyz, j.originRpy);

        // joint rotation
        if (j.type == JointType::Revolute || j.type == JointType::Continuous) {
            float q = robot_.GetJointAngle(j.name);
            childWorld = childWorld * glm::rotate(glm::mat4(1.0f), q, j.axis);
        }

        DrawLinkRecursive(j.childLink, childWorld, shader);
    }
}
