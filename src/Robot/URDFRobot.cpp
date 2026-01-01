#include "URDFRobot.h"

#include <tinyxml2.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <unordered_map>
#include <cstring>   // std::strlen

// Adjust these include paths to wherever YOU placed them
#include "../Renderer/Mesh.h"
#include "../Renderer/STLLoader.h"
#include "../Renderer/FeatureEdges.h"

static glm::vec3 parseVec3(const char* s) {
    if (!s) return {0, 0, 0};
    std::istringstream iss(s);
    glm::vec3 v(0.0f);
    iss >> v.x >> v.y >> v.z;
    return v;
}

static std::string resolvePackageMesh(const std::string& urdfFilename,
                                      const std::string& packageMeshesRoot) {
    constexpr const char* prefix = "package://arm_pkg/description/meshes/";
    if (urdfFilename.rfind(prefix, 0) == 0) {
        return packageMeshesRoot + "/" + urdfFilename.substr(std::strlen(prefix));
    }
    return urdfFilename;
}

static JointType parseJointType(const char* t) {
    if (!t) return JointType::Fixed;
    std::string s(t);
    if (s == "fixed") return JointType::Fixed;
    if (s == "revolute") return JointType::Revolute;
    if (s == "continuous") return JointType::Continuous;
    return JointType::Fixed;
}

bool URDFRobot::LoadFromFile(const std::string& urdfPath, const std::string& packageMeshesRoot) {       
    links_.clear();
    joints_.clear();
    childrenByLink_.clear();
    jointAngles_.clear();
    rootLink_.clear();

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(urdfPath.c_str()) != tinyxml2::XML_SUCCESS) {
        return false;
    }

    auto* robot = doc.FirstChildElement("robot");
    if (!robot) return false;

    // --- links ---
    for (auto* linkEl = robot->FirstChildElement("link");
         linkEl; linkEl = linkEl->NextSiblingElement("link")) {

        URDFLink link;
        link.name = linkEl->Attribute("name") ? linkEl->Attribute("name") : "";

        auto* visualEl = linkEl->FirstChildElement("visual");
        if (visualEl) {
            link.hasVisual = true;

            if (auto* originEl = visualEl->FirstChildElement("origin")) {
                link.visual.xyz = parseVec3(originEl->Attribute("xyz"));
                link.visual.rpy = parseVec3(originEl->Attribute("rpy"));
            }

            auto* geomEl = visualEl->FirstChildElement("geometry");
            if (geomEl) {
                auto* meshEl = geomEl->FirstChildElement("mesh");
                if (meshEl && meshEl->Attribute("filename")) {
                    link.visual.meshPath = resolvePackageMesh(meshEl->Attribute("filename"),
                                                             packageMeshesRoot);
                    link.visual.scale = parseVec3(meshEl->Attribute("scale"));
                    if (link.visual.scale == glm::vec3(0.0f)) link.visual.scale = {1,1,1};
                }
            }

            if (!link.visual.meshPath.empty()) {
                auto stl = STLLoader::LoadBinary(link.visual.meshPath);

                // Fill mesh (triangles)
                const float r = 0.70f, g = 0.70f, b = 0.70f; // silver
                std::vector<float> interleaved;
                interleaved.reserve(stl.positions.size() * 6);

                for (const auto& p : stl.positions) {
                    interleaved.push_back(p.x);
                    interleaved.push_back(p.y);
                    interleaved.push_back(p.z);
                    interleaved.push_back(r);
                    interleaved.push_back(g);
                    interleaved.push_back(b);
                }

                link.mesh = std::make_shared<Mesh>(
                    Mesh::fromVertexColorTriangles(interleaved)
                );

                // Feature edges (CAD-like)
                const glm::vec3 edgeCol(0.10f, 0.12f, 0.18f);
                float weldEps   = 0.02f;   // tune if needed
                float creaseDeg = 35.0f;   // tune if needed

                auto edgeLines = BuildFeatureEdgeLines(stl, weldEps, creaseDeg, edgeCol);
                if (!edgeLines.empty()) {
                    link.edgeMesh = std::make_shared<Mesh>(
                        Mesh::fromVertexColorLines(edgeLines)
                    );
                }
            }
        }

        links_[link.name] = std::move(link);
    }

    // --- joints ---
    for (auto* jointEl = robot->FirstChildElement("joint");
         jointEl; jointEl = jointEl->NextSiblingElement("joint")) {

        URDFJoint j;
        j.name = jointEl->Attribute("name") ? jointEl->Attribute("name") : "";
        j.type = parseJointType(jointEl->Attribute("type"));

        if (auto* parentEl = jointEl->FirstChildElement("parent")) {
            j.parentLink = parentEl->Attribute("link") ? parentEl->Attribute("link") : "";
        }
        if (auto* childEl = jointEl->FirstChildElement("child")) {
            j.childLink = childEl->Attribute("link") ? childEl->Attribute("link") : "";
        }
        if (auto* originEl = jointEl->FirstChildElement("origin")) {
            j.originXyz = parseVec3(originEl->Attribute("xyz"));
            j.originRpy = parseVec3(originEl->Attribute("rpy"));
        }
        if (auto* axisEl = jointEl->FirstChildElement("axis")) {
            j.axis = parseVec3(axisEl->Attribute("xyz"));
            if (glm::length(j.axis) > 0.0001f) j.axis = glm::normalize(j.axis);
        }
        if (auto* limitEl = jointEl->FirstChildElement("limit")) {
            if (limitEl->Attribute("lower") && limitEl->Attribute("upper")) {
                j.hasLimits = true;
                j.lower = std::stof(limitEl->Attribute("lower"));
                j.upper = std::stof(limitEl->Attribute("upper"));
            }
        }

        const int idx = (int)joints_.size();
        joints_.push_back(j);
        childrenByLink_[j.parentLink].push_back(idx);
        jointAngles_[j.name] = 0.0f;
    }

    // find root link (never a child)
    std::unordered_map<std::string, bool> isChild;
    for (const auto& kv : links_) isChild[kv.first] = false;
    for (const auto& j : joints_) isChild[j.childLink] = true;

    for (const auto& kv : isChild) {
        if (!kv.second) { rootLink_ = kv.first; break; }
    }
    if (rootLink_.empty() && !links_.empty())
        rootLink_ = links_.begin()->first;

    return true;
}

void URDFRobot::SetJointAngle(const std::string& jointName, float radians) {
    jointAngles_[jointName] = radians;
}

float URDFRobot::GetJointAngle(const std::string& jointName) const {
    auto it = jointAngles_.find(jointName);
    return (it == jointAngles_.end()) ? 0.0f : it->second;
}

const std::vector<int>& URDFRobot::ChildJointsOf(const std::string& linkName) const {
    static const std::vector<int> empty;
    auto it = childrenByLink_.find(linkName);
    return (it == childrenByLink_.end()) ? empty : it->second;
}
