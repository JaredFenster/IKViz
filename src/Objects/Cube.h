#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../Robot/RobotScene.h"

class Shader;                // forward declare
namespace URDFIK { struct ChainInfo; } // forward declare

class Cube
{
public:
    Cube(const glm::vec3& position,
         const glm::quat& rotation,
         const glm::vec3& scale);

    // If grabbed == true, cube will follow the parent's end effector pose
    void SetGrabbed(bool g);

    // Call this once when you want the cube to follow a robot
    // (you provide the robot + its chain used for FK)
    void SetParent(RobotScene* parent, const URDFIK::ChainInfo* chain);

    // Call every frame (or only while grabbed, your choice)
    void UpdateFromParentEE();

    void Draw(Shader& shader) const;

public:
    glm::vec3 position{0.0f};
    glm::quat rotation{1,0,0,0};
    glm::vec3 scale{1.0f};

    glm::vec3 pickVector{0.0f}; // kept for later
    bool grabbed = false;
    // kept for later (not used now)
    float gravity = 9.8f;
    float groundPlaneOffset = 0.0f;

private:
    RobotScene* parent_ = nullptr;
    const URDFIK::ChainInfo* chain_ = nullptr;
};
