#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <glm/glm.hpp>

class Mesh;

struct URDFVisual {
    glm::vec3 xyz{0};
    glm::vec3 rpy{0};          // roll, pitch, yaw (radians)
    glm::vec3 scale{1.0f};
    std::string meshPath;      // resolved filesystem path
};

enum class JointType { Fixed, Revolute, Continuous };

struct URDFJoint {
    std::string name;
    JointType type{JointType::Fixed};

    std::string parentLink;
    std::string childLink;

    glm::vec3 originXyz{0};
    glm::vec3 originRpy{0};
    glm::vec3 axis{0,0,1};

    bool hasLimits{false};
    float lower{0}, upper{0};
};

struct URDFLink {
    std::string name;
    bool hasVisual{false};
    URDFVisual visual;

    std::shared_ptr<Mesh> mesh;      
    std::shared_ptr<Mesh> edgeMesh;  
};


class URDFRobot {
public:
    bool LoadFromFile(const std::string& urdfPath,
                      const std::string& packageMeshesRoot); // build/robot/meshes

    // joint name -> angle radians
    void SetJointAngle(const std::string& jointName, float radians);
    float GetJointAngle(const std::string& jointName) const;

    const std::string& RootLink() const { return rootLink_; }

    // rendering traversal info
    const std::unordered_map<std::string, URDFLink>& Links() const { return links_; }
    const std::vector<URDFJoint>& Joints() const { return joints_; }

    // convenience: child joints by parent link
    const std::vector<int>& ChildJointsOf(const std::string& linkName) const;

    void setPos(glm::vec3 pos) {this->pos = pos;};
    glm::vec3 getPos() {return pos;}


private:
    glm::vec3 pos;
    std::string rootLink_;
    std::unordered_map<std::string, URDFLink> links_;
    std::vector<URDFJoint> joints_;
    std::unordered_map<std::string, float> jointAngles_;
    std::unordered_map<std::string, std::vector<int>> childrenByLink_;
};
