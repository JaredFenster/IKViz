#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class linkJoint;
class Origin;

// 6D Jacobian column: [linear; angular]
struct JCol6 {
    glm::vec3 v; // linear
    glm::vec3 w; // angular
};

class IK {
public:
    explicit IK(linkJoint& robot);

    // Position-only IK
    bool solvePosition(
        const glm::vec3& targetWorld,
        int endEffectorIndex = -1,
        int maxIterations = 30,
        float tolerance = 1e-3f,
        float lambda = 0.15f,
        float maxStepDeg = 2.0f
    );

    // Pose IK (position + orientation)
    bool solvePose(
        const glm::vec3& targetPosWorld,
        const glm::quat& targetRotWorld,
        int endEffectorIndex = -1,
        int maxIterations = 30,
        float posTolerance = 1e-3f,
        float rotToleranceRad = 1e-2f,
        float lambda = 0.15f,
        float maxStepDeg = 2.0f,
        float rotWeight = 1.0f
    );

private:
    linkJoint& robot_;

    static float clampf(float v, float lo, float hi);

    // Access robot chain (requires friend class IK in linkJoint)
    const std::vector<Origin*>& chain_() const;

    // Build Jacobians
    void buildJacobian(
        const std::vector<Origin*>& chain,
        int endIdx,
        const glm::vec3& pEE,
        std::vector<glm::vec3>& Jcols
    ) const;

    void buildJacobianPose(
        const std::vector<Origin*>& chain,
        int endIdx,
        const glm::vec3& pEE,
        std::vector<JCol6>& Jcols
    ) const;

    // Task-space DLS solvers (KDL-style):
    // dq = J^T (J J^T + λ² I)^-1 e
    bool solveDLS_Task3(
        const std::vector<glm::vec3>& Jcols,
        const glm::vec3& e,
        float lambda,
        std::vector<float>& outDeltaThetaRad
    ) const;

    bool solveDLS_Task6(
        const std::vector<JCol6>& Jcols,
        const glm::vec3& ep,
        const glm::vec3& er,
        float lambda,
        float rotWeight,
        std::vector<float>& outDeltaThetaRad
    ) const;

    // Generic Gauss-Jordan solver (NxN)
    bool solveLinearSystem(
        std::vector<std::vector<float>>& A,
        std::vector<float>& b
    ) const;

    // Apply 1 joint update with limits and propagate to downstream joints
    void applyDeltaDeg(
        const std::vector<Origin*>& chain,
        int jointIdx,
        float deltaDeg,
        int endIdx
    ) const;

    // Helpers for stable iteration
    void snapshotAngles(
        const std::vector<Origin*>& chain,
        int endIdx,
        std::vector<float>& outAnglesDeg
    ) const;

    void restoreAngles(
        const std::vector<Origin*>& chain,
        int endIdx,
        const std::vector<float>& anglesDeg
    ) const;
};
