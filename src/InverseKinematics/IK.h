// IK.h
#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <vector>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class URDFRobot;

namespace URDFIK {

struct ChainInfo {
    std::vector<int> jointIdx;
    std::string eeLink;
};

struct FKResult {
    glm::vec3 pos{0.0f};
    glm::quat rot{1.0f, 0.0f, 0.0f, 0.0f};
};

// Picks a simple serial chain by walking "first child joint" from RootLink() to a leaf.
// (Same logic you had in App.cpp.)
ChainInfo BuildSerialChain(const URDFRobot& urdf);

// Computes world pose of the end effector for the provided chain.
// Also outputs the world transform of each joint frame BEFORE applying the joint motion.
FKResult ComputeFK(
    const URDFRobot& urdf,
    const ChainInfo& chain,
    std::vector<glm::mat4>& outJointFrameWorld
);

// Coupled 6D task-space DLS pose IK (your current best solver), including:
// - axis-angle orientation error in world
// - rotation-weight ramp based on position error
// - global step clamp (deg)
// - line search + adaptive damping
bool SolvePoseHierDLS(
    URDFRobot& urdf,
    const ChainInfo& chain,
    const glm::vec3& targetPosWorld,
    const glm::quat& targetRotWorld,
    int   maxIterations   = 60,
    float posTolerance    = 0.002f,
    float rotToleranceRad = 0.02f,
    float maxStepDeg      = 6.0f,
    float rotWeight       = 1.0f,
    float lambda          = 0.20f
);

} // namespace URDFIK
