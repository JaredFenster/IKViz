#pragma once
#include <glm/glm.hpp>
#include <vector>

class linkJoint;
class Origin;

class IK {
public:
    explicit IK(linkJoint& robot);

    // Position-only IK. End effector = robot[endIdx]->getPos()
    // Returns true if converged.
    bool solvePosition(
        const glm::vec3& targetWorld,
        int endEffectorIndex = -1,
        int maxIterations = 25,
        float tolerance = 1e-3f,
        float lambda = 0.10f,     // damping
        float maxStepDeg = 4.0f   // per-joint update clamp
    );

private:
    linkJoint& robot_;

    // Convenience
    const std::vector<Origin*>& chain_() const;

    // J columns: Ji = w_i x (pEE - p_i)
    void buildJacobian(
        const std::vector<Origin*>& chain,
        int endIdx,
        const glm::vec3& pEE,
        std::vector<glm::vec3>& Jcols
    ) const;

    // DLS: dθ = (JᵀJ + λ²I)^-1 Jᵀ e
    bool solveDLS(
        const std::vector<glm::vec3>& Jcols,
        const glm::vec3& e,
        float lambda,
        std::vector<float>& outDeltaThetaRad
    ) const;

    // Solve A x = b (Gauss-Jordan)
    bool solveLinearSystem(
        std::vector<std::vector<float>>& A,
        std::vector<float>& b
    ) const;

    // Apply rotation at joint i and propagate to downstream joints
    void applyDeltaDeg(
        const std::vector<Origin*>& chain,
        int jointIdx,
        float deltaDeg,
        int endIdx
    ) const;

    static float clampf(float v, float lo, float hi);
};
