#define GLM_ENABLE_EXPERIMENTAL
#include "IK.h"
#include "../Linker/linkJoint.h"
#include "../Linker/origin.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <cmath>
#include <algorithm>

static glm::vec3 orientationErrorAxisAngle(const glm::quat& qCurrent,
                                           const glm::quat& qTarget)
{
    // error that rotates current -> target
    glm::quat qErr = qTarget * glm::conjugate(qCurrent);
    qErr = glm::normalize(qErr);

    // Keep shortest rotation
    if (qErr.w < 0.0f) qErr = -qErr;

    float w = glm::clamp(qErr.w, -1.0f, 1.0f);
    float angle = 2.0f * std::acos(w);          // [0, pi]
    float s = std::sqrt(std::max(0.0f, 1.0f - w*w));

    if (s < 1e-8f || angle < 1e-8f) {
        return glm::vec3(0.0f);
    }

    glm::vec3 axis(qErr.x / s, qErr.y / s, qErr.z / s);
    return axis * angle; // radians
}

float IK::clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

IK::IK(linkJoint& robot) : robot_(robot) {}

const std::vector<Origin*>& IK::chain_() const {
    // Requires: friend class IK; in linkJoint
    return robot_.robot;
}

void IK::buildJacobian(
    const std::vector<Origin*>& chain,
    int endIdx,
    const glm::vec3& pEE,
    std::vector<glm::vec3>& Jcols
) const {
    const int n = endIdx + 1;
    Jcols.assign(n, glm::vec3(0.0f));

    for (int i = 0; i < n; ++i) {
        const glm::vec3 pi = chain[i]->getPos();
        const glm::vec3 wi = glm::normalize(chain[i]->getJointAxisWorld());
        Jcols[i] = glm::cross(wi, (pEE - pi));
    }
}

void IK::buildJacobianPose(
    const std::vector<Origin*>& chain,
    int endIdx,
    const glm::vec3& pEE,
    std::vector<JCol6>& Jcols
) const {
    const int n = endIdx + 1;
    Jcols.assign(n, {glm::vec3(0), glm::vec3(0)});

    for (int i = 0; i < n; ++i) {
        const glm::vec3 pi = chain[i]->getPos();
        const glm::vec3 wi = glm::normalize(chain[i]->getJointAxisWorld());
        Jcols[i].v = glm::cross(wi, (pEE - pi));
        Jcols[i].w = wi;
    }
}

bool IK::solveLinearSystem(
    std::vector<std::vector<float>>& A,
    std::vector<float>& b
) const {
    const int n = (int)b.size();
    if ((int)A.size() != n) return false;

    for (int col = 0; col < n; ++col) {
        int pivot = col;
        float best = std::fabs(A[pivot][col]);
        for (int r = col + 1; r < n; ++r) {
            float v = std::fabs(A[r][col]);
            if (v > best) { best = v; pivot = r; }
        }
        if (best < 1e-10f) return false;

        if (pivot != col) {
            std::swap(A[pivot], A[col]);
            std::swap(b[pivot], b[col]);
        }

        const float diag = A[col][col];
        for (int c = col; c < n; ++c) A[col][c] /= diag;
        b[col] /= diag;

        for (int r = 0; r < n; ++r) {
            if (r == col) continue;
            const float f = A[r][col];
            if (std::fabs(f) < 1e-12f) continue;
            for (int c = col; c < n; ++c) A[r][c] -= f * A[col][c];
            b[r] -= f * b[col];
        }
    }
    return true;
}

// ---------------------------
// KDL-style task-space DLS (3D)
// dq = J^T (J J^T + λ² I)^-1 e
// ---------------------------
bool IK::solveDLS_Task3(
    const std::vector<glm::vec3>& Jcols,
    const glm::vec3& e,
    float lambda,
    std::vector<float>& outDeltaThetaRad
) const {
    const int n = (int)Jcols.size();
    outDeltaThetaRad.assign(n, 0.0f);

    // M = J J^T (3x3)
    std::vector<std::vector<float>> M(3, std::vector<float>(3, 0.0f));

    for (int i = 0; i < n; ++i) {
        const glm::vec3& Ji = Jcols[i];
        M[0][0] += Ji.x * Ji.x; M[0][1] += Ji.x * Ji.y; M[0][2] += Ji.x * Ji.z;
        M[1][0] += Ji.y * Ji.x; M[1][1] += Ji.y * Ji.y; M[1][2] += Ji.y * Ji.z;
        M[2][0] += Ji.z * Ji.x; M[2][1] += Ji.z * Ji.y; M[2][2] += Ji.z * Ji.z;
    }

    const float lam2 = lambda * lambda;
    M[0][0] += lam2; M[1][1] += lam2; M[2][2] += lam2;

    std::vector<float> b = { e.x, e.y, e.z };   // solve for y
    if (!solveLinearSystem(M, b)) return false; // b becomes y

    const glm::vec3 y(b[0], b[1], b[2]);

    // dq = J^T y  (dq_i = Ji · y)
    for (int i = 0; i < n; ++i) {
        outDeltaThetaRad[i] = glm::dot(Jcols[i], y);
    }

    return true;
}

// ---------------------------
// KDL-style task-space DLS (6D)
// dq = J^T (J J^T + λ² I)^-1 e
// with row-scaling for rotation weight
// ---------------------------
bool IK::solveDLS_Task6(
    const std::vector<JCol6>& Jcols,
    const glm::vec3& ep,
    const glm::vec3& er,
    float lambda,
    float rotWeight,
    std::vector<float>& outDeltaThetaRad
) const {
    const int n = (int)Jcols.size();
    outDeltaThetaRad.assign(n, 0.0f);

    // Build e6 = [ep; rotWeight*er]
    std::vector<float> e6 = {
        ep.x, ep.y, ep.z,
        rotWeight * er.x, rotWeight * er.y, rotWeight * er.z
    };

    // M = J J^T (6x6), where each column is [v; rotWeight*w]
    std::vector<std::vector<float>> M(6, std::vector<float>(6, 0.0f));

    for (int i = 0; i < n; ++i) {
        const glm::vec3 v = Jcols[i].v;
        const glm::vec3 w = rotWeight * Jcols[i].w;

        float Ji[6] = { v.x, v.y, v.z, w.x, w.y, w.z };

        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < 6; ++c) {
                M[r][c] += Ji[r] * Ji[c];
            }
        }
    }

    const float lam2 = lambda * lambda;
    for (int d = 0; d < 6; ++d) M[d][d] += lam2;

    if (!solveLinearSystem(M, e6)) return false; // e6 becomes y

    // dq_i = Ji · y
    for (int i = 0; i < n; ++i) {
        const glm::vec3 v = Jcols[i].v;
        const glm::vec3 w = rotWeight * Jcols[i].w;
        float Ji[6] = { v.x, v.y, v.z, w.x, w.y, w.z };

        float sum = 0.0f;
        for (int k = 0; k < 6; ++k) sum += Ji[k] * e6[k];
        outDeltaThetaRad[i] = sum;
    }

    return true;
}

void IK::applyDeltaDeg(
    const std::vector<Origin*>& chain,
    int jointIdx,
    float deltaDeg,
    int endIdx
) const {
    Origin* pivot = chain[jointIdx];
    const int pivotAxisInt = pivot->getRotateAxisInt();

    const float current = pivot->getAngleDeg();
    const float desired = current + deltaDeg;

    const float clamped = pivot->clampToLimits(desired);
    const float applied = clamped - current;

    if (std::fabs(applied) < 1e-9f) return;

    pivot->setAngleDeg(clamped);

    for (int k = jointIdx + 1; k <= endIdx; ++k) {
        chain[k]->rotateAboutOtherOriginAxis(*pivot, pivotAxisInt, applied, true);
    }
}

void IK::snapshotAngles(
    const std::vector<Origin*>& chain,
    int endIdx,
    std::vector<float>& outAnglesDeg
) const {
    outAnglesDeg.resize(endIdx + 1);
    for (int i = 0; i <= endIdx; ++i) outAnglesDeg[i] = chain[i]->getAngleDeg();
}

void IK::restoreAngles(
    const std::vector<Origin*>& chain,
    int endIdx,
    const std::vector<float>& anglesDeg
) const {
    for (int i = 0; i <= endIdx; ++i) {
        float cur = chain[i]->getAngleDeg();
        float back = anglesDeg[i] - cur;
        if (std::fabs(back) > 1e-9f) {
            applyDeltaDeg(chain, i, back, endIdx);
        }
    }
}

bool IK::solvePosition(
    const glm::vec3& targetWorld,
    int endEffectorIndex,
    int maxIterations,
    float tolerance,
    float lambda,
    float maxStepDeg
) {
    const auto& chain = chain_();
    if (chain.empty()) return false;

    int endIdx = endEffectorIndex;
    if (endIdx < 0) endIdx = (int)chain.size() - 1;
    endIdx = std::max(0, std::min(endIdx, (int)chain.size() - 1));

    std::vector<glm::vec3> Jcols;
    std::vector<float> dThetaRad;

    float lambdaLocal = std::max(1e-6f, lambda);

    for (int iter = 0; iter < maxIterations; ++iter) {
        const glm::vec3 pEE = chain[endIdx]->getPos();
        const glm::vec3 e = targetWorld - pEE;
        float err0 = glm::length(e);

        if (err0 < tolerance) return true;

        buildJacobian(chain, endIdx, pEE, Jcols);
        if (!solveDLS_Task3(Jcols, e, lambdaLocal, dThetaRad)) return false;

        // Convert to degrees and globally scale step so the largest joint step is maxStepDeg
        std::vector<float> stepDeg(endIdx + 1, 0.0f);
        float maxAbs = 0.0f;
        for (int i = 0; i <= endIdx; ++i) {
            float dDeg = glm::degrees(dThetaRad[i]);
            stepDeg[i] = dDeg;
            maxAbs = std::max(maxAbs, std::fabs(dDeg));
        }
        if (maxAbs > maxStepDeg && maxAbs > 1e-9f) {
            float s = maxStepDeg / maxAbs;
            for (float& v : stepDeg) v *= s;
        }

        // Line search: only accept steps that reduce error
        std::vector<float> snap;
        snapshotAngles(chain, endIdx, snap);

        bool accepted = false;
        float alpha = 1.0f;
        for (int ls = 0; ls < 5; ++ls) {
            for (int i = 0; i <= endIdx; ++i) {
                if (std::fabs(stepDeg[i]) > 1e-7f)
                    applyDeltaDeg(chain, i, alpha * stepDeg[i], endIdx);
            }

            float err1 = glm::length(targetWorld - chain[endIdx]->getPos());
            if (err1 < err0) {
                accepted = true;
                break;
            }

            // revert and try smaller step
            restoreAngles(chain, endIdx, snap);
            alpha *= 0.5f;
        }

        if (!accepted) {
            // stuck / oscillating -> increase damping (more stable, smaller effective steps)
            lambdaLocal = std::min(lambdaLocal * 2.0f, 10.0f);
        } else {
            // converging -> reduce damping for speed
            lambdaLocal = std::max(lambdaLocal * 0.7f, 1e-6f);
        }
    }

    return false;
}

bool IK::solvePose(
    const glm::vec3& targetPosWorld,
    const glm::quat& targetRotWorld,
    int endEffectorIndex,
    int maxIterations,
    float posTolerance,
    float rotToleranceRad,
    float lambda,
    float maxStepDeg,
    float rotWeight
) {
    const auto& chain = chain_();
    if (chain.empty()) return false;

    int endIdx = endEffectorIndex;
    if (endIdx < 0) endIdx = (int)chain.size() - 1;
    endIdx = std::max(0, std::min(endIdx, (int)chain.size() - 1));

    std::vector<JCol6> Jcols;
    std::vector<float> dThetaRad;

    float lambdaLocal = std::max(1e-6f, lambda);
    float rw = std::max(0.0f, rotWeight);

    for (int iter = 0; iter < maxIterations; ++iter) {
        const glm::vec3 pEE = chain[endIdx]->getPos();
        const glm::vec3 ep = targetPosWorld - pEE;

        glm::quat qEE = chain[endIdx]->getRotationQuat();
        glm::vec3 er = orientationErrorAxisAngle(qEE, targetRotWorld);

        float posErr0 = glm::length(ep);
        float rotErr0 = glm::length(er);

        if (posErr0 < posTolerance && rotErr0 < rotToleranceRad) return true;

        // combined metric for line-search acceptance
        float metric0 = std::sqrt(posErr0 * posErr0 + (rw * rotErr0) * (rw * rotErr0));

        buildJacobianPose(chain, endIdx, pEE, Jcols);

        if (!solveDLS_Task6(Jcols, ep, er, lambdaLocal, rw, dThetaRad)) return false;

        // Convert to degrees and globally scale
        std::vector<float> stepDeg(endIdx + 1, 0.0f);
        float maxAbs = 0.0f;
        for (int i = 0; i <= endIdx; ++i) {
            float dDeg = glm::degrees(dThetaRad[i]);
            stepDeg[i] = dDeg;
            maxAbs = std::max(maxAbs, std::fabs(dDeg));
        }
        if (maxAbs > maxStepDeg && maxAbs > 1e-9f) {
            float s = maxStepDeg / maxAbs;
            for (float& v : stepDeg) v *= s;
        }

        // Line search
        std::vector<float> snap;
        snapshotAngles(chain, endIdx, snap);

        bool accepted = false;
        float alpha = 1.0f;
        for (int ls = 0; ls < 5; ++ls) {
            for (int i = 0; i <= endIdx; ++i) {
                if (std::fabs(stepDeg[i]) > 1e-7f)
                    applyDeltaDeg(chain, i, alpha * stepDeg[i], endIdx);
            }

            const glm::vec3 p1 = chain[endIdx]->getPos();
            const glm::vec3 ep1 = targetPosWorld - p1;
            glm::quat q1 = chain[endIdx]->getRotationQuat();
            glm::vec3 er1 = orientationErrorAxisAngle(q1, targetRotWorld);

            float metric1 = std::sqrt(glm::dot(ep1, ep1) + (rw * glm::length(er1)) * (rw * glm::length(er1)));

            if (metric1 < metric0) {
                accepted = true;
                break;
            }

            restoreAngles(chain, endIdx, snap);
            alpha *= 0.5f;
        }

        if (!accepted) {
            lambdaLocal = std::min(lambdaLocal * 2.0f, 10.0f);
        } else {
            lambdaLocal = std::max(lambdaLocal * 0.7f, 1e-6f);
        }
    }

    return false;
}
