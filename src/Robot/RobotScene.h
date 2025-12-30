#pragma once
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>


#include "../Linker/origin.h"
#include "../Linker/linkJoint.h"
#include "../InverseKinematics/IK.h"

class Shader;
class Mesh;

class RobotScene {
public:
    explicit RobotScene(const std::string& jsonPath);

    void reset();
    void uiAndSolve();
    void draw(const Shader& shader, const Mesh& sphereWire);
    glm::vec3 eePos(){ return origins_.back().getPos(); }
    glm::quat eeRot(){ return origins_.back().getRotationQuat(); }
    void setIKTarget(const glm::vec3& pos, const glm::quat& rot);


private:
    void loadFromJson(const std::string& path);

private:
    std::vector<Origin> origins_;
    std::unique_ptr<linkJoint> robot_;
    std::unique_ptr<IK> ik_;
    Origin* end_ = nullptr;

    // IK params + targets
    bool  ikEnabled_ = true;
    int   itersPerFrame_ = 2;
    float lambda_ = 0.15f;
    float maxStepDeg_ = 2.0f;
    float rotWeight_ = 1.0f;

    glm::vec3 targetWorld_ = glm::vec3(2.0f, 0.0f, 2.0f);
    glm::vec3 targetEulerDeg_ = glm::vec3(0.0f);
};
