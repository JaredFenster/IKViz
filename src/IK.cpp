#include "IK.h"
#include "linkJoint.h"
#include "origin.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <algorithm>

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

bool IK::solveDLS(
    const std::vector<glm::vec3>& Jcols,
    const glm::vec3& e,
    float lambda,
    std::vector<float>& outDeltaThetaRad
) const {
    const int n = (int)Jcols.size();
    outDeltaThetaRad.assign(n, 0.0f);

    std::vector<std::vector<float>> A(n, std::vector<float>(n, 0.0f));
    std::vector<float> b(n, 0.0f);

    for (int i = 0; i < n; ++i) {
        b[i] = glm::dot(Jcols[i], e);
        for (int j = 0; j < n; ++j) {
            A[i][j] = glm::dot(Jcols[i], Jcols[j]);
        }
    }

    const float lam2 = lambda * lambda;
    for (int i = 0; i < n; ++i) A[i][i] += lam2;

    if (!solveLinearSystem(A, b)) return false;

    outDeltaThetaRad = b; // radians
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

    // Rotate pivot about its own axis (orientation changes, position stays)
    pivot->rotate(deltaDeg);

    // Propagate: rotate each downstream origin about pivot's axis, and rotate its position around pivot
    for (int k = jointIdx + 1; k <= endIdx; ++k) {
        chain[k]->rotateAboutOtherOriginAxis(*pivot, pivotAxisInt, deltaDeg, /*rotatePosition=*/true);
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

    for (int iter = 0; iter < maxIterations; ++iter) {
        const glm::vec3 pEE = chain[endIdx]->getPos();
        const glm::vec3 e = targetWorld - pEE;
        const float err = glm::length(e);
        if (err < tolerance) return true;

        buildJacobian(chain, endIdx, pEE, Jcols);
        if (!solveDLS(Jcols, e, lambda, dThetaRad)) return false;

        for (int i = 0; i <= endIdx; ++i) {
            float dDeg = glm::degrees(dThetaRad[i]);
            dDeg = clampf(dDeg, -maxStepDeg, maxStepDeg);
            if (std::fabs(dDeg) < 1e-7f) continue;

            applyDeltaDeg(chain, i, dDeg, endIdx);
        }
    }

    return false;
}
