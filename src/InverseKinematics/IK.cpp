// IK.cpp
#define GLM_ENABLE_EXPERIMENTAL
#include "IK.h"

#include "../Robot/URDFRobot.h"
#include "../Robot/URDFMath.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>

namespace URDFIK {

static float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

// -------------------------
// Generic Gauss-Jordan (NxN)
// -------------------------
static bool solveLinearSystemNxN(std::vector<std::vector<float>>& A,
                                std::vector<float>& b)
{
    const int n = (int)b.size();
    if ((int)A.size() != n) return false;
    for (int r = 0; r < n; ++r) if ((int)A[r].size() != n) return false;

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

// 6D Jacobian column: [linear; angular]
struct JCol6 {
    glm::vec3 v;
    glm::vec3 w;
};

// ---------------------------
// KDL-style task-space DLS (6D)
// dq = J^T (J J^T + λ² I)^-1 e
// where each column is [v; rotWeight*w]
// ---------------------------
static bool solveTaskDLS6(
    const std::vector<JCol6>& Jcols,
    const glm::vec3& ep,
    const glm::vec3& er,
    float lambda,
    float rotWeight,
    std::vector<float>& outDQ
) {
    const int n = (int)Jcols.size();
    outDQ.assign(n, 0.0f);

    // e6 = [ep; rotWeight*er]
    std::vector<float> e6 = {
        ep.x, ep.y, ep.z,
        rotWeight * er.x, rotWeight * er.y, rotWeight * er.z
    };

    // M = J J^T (6x6)
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

    // Solve M y = e6  (e6 becomes y)
    if (!solveLinearSystemNxN(M, e6)) return false;

    // dq_i = Ji · y
    for (int i = 0; i < n; ++i) {
        const glm::vec3 v = Jcols[i].v;
        const glm::vec3 w = rotWeight * Jcols[i].w;
        float Ji[6] = { v.x, v.y, v.z, w.x, w.y, w.z };

        float sum = 0.0f;
        for (int k = 0; k < 6; ++k) sum += Ji[k] * e6[k];
        outDQ[i] = sum;
    }

    return true;
}

static void snapshotChainAngles(const URDFRobot& urdf, const ChainInfo& chain, std::vector<float>& out) {
    out.resize(chain.jointIdx.size());
    for (size_t k = 0; k < chain.jointIdx.size(); ++k) {
        const URDFJoint& j = urdf.Joints()[chain.jointIdx[k]];
        out[k] = urdf.GetJointAngle(j.name);
    }
}

static void restoreChainAngles(URDFRobot& urdf, const ChainInfo& chain, const std::vector<float>& snap) {
    for (size_t k = 0; k < chain.jointIdx.size(); ++k) {
        const URDFJoint& j = urdf.Joints()[chain.jointIdx[k]];
        urdf.SetJointAngle(j.name, snap[k]);
    }
}

static glm::vec3 quatErrorAxisAngleWorld(const glm::quat& qCurrentWorld,
                                        const glm::quat& qTargetWorld,
                                        float* outAngleMag = nullptr)
{
    glm::quat qErr = glm::normalize(qTargetWorld * glm::inverse(qCurrentWorld));
    if (qErr.w < 0.0f) qErr = -qErr;

    glm::vec3 v(qErr.x, qErr.y, qErr.z);
    float s = glm::length(v);

    float ang = 2.0f * std::atan2(s, clampf(qErr.w, -1.0f, 1.0f));
    if (outAngleMag) *outAngleMag = std::fabs(ang);

    if (s < 1e-8f || std::fabs(ang) < 1e-8f) return glm::vec3(0.0f);
    glm::vec3 axis = v / s;
    return axis * ang; // radians (world)
}

// ------------------- Public API -------------------

ChainInfo BuildSerialChain(const URDFRobot& urdf) {
    ChainInfo c;
    std::string cur = urdf.RootLink();

    for (int safety = 0; safety < 64; ++safety) {
        const auto& childs = urdf.ChildJointsOf(cur);
        if (childs.empty()) {
            c.eeLink = cur;
            break;
        }
        int jIdx = childs[0];
        c.jointIdx.push_back(jIdx);
        cur = urdf.Joints()[jIdx].childLink;
    }

    if (c.eeLink.empty()) c.eeLink = cur;
    return c;
}

FKResult ComputeFK(
    const URDFRobot& urdf,
    const ChainInfo& chain,
    std::vector<glm::mat4>& outJointFrameWorld
) {
    outJointFrameWorld.clear();
    outJointFrameWorld.reserve(chain.jointIdx.size());

    glm::mat4 T = glm::mat4(1.0f);

    for (int idx : chain.jointIdx) {
        const URDFJoint& j = urdf.Joints()[idx];

        glm::mat4 T_origin = urdfXYZRPY(j.originXyz, j.originRpy);

        glm::mat4 T_jointFrame = T * T_origin;
        outJointFrameWorld.push_back(T_jointFrame);

        float q = urdf.GetJointAngle(j.name);
        if (j.type == JointType::Revolute || j.type == JointType::Continuous) {
            T = T_jointFrame * glm::rotate(glm::mat4(1.0f), q, j.axis);
        } else {
            T = T_jointFrame;
        }
    }

    FKResult out;
    out.pos = glm::vec3(T * glm::vec4(0,0,0,1));
    out.rot = glm::normalize(glm::quat_cast(glm::mat3(T)));
    return out;
}

bool SolvePoseHierDLS(
    URDFRobot& urdf,
    const ChainInfo& chain,
    const glm::vec3& targetPosWorld,
    const glm::quat& targetRotWorld,
    int maxIterations,
    float posTolerance,
    float rotToleranceRad,
    float maxStepDeg,
    float rotWeight,
    float lambda
) {
    if (chain.jointIdx.empty()) return false;

    const float maxStepRad = glm::radians(maxStepDeg);
    const int n = (int)chain.jointIdx.size();

    std::vector<glm::mat4> jointFrames;
    std::vector<JCol6> Jcols(n);

    std::vector<float> dq(n, 0.0f);

    float lambdaLocal = std::max(1e-6f, lambda);
    const glm::quat qTargetN = glm::normalize(targetRotWorld);

    for (int it = 0; it < maxIterations; ++it) {
        FKResult fk = ComputeFK(urdf, chain, jointFrames);

        glm::vec3 ep = targetPosWorld - fk.pos;
        float posErr0 = glm::length(ep);

        float angErr0 = 0.0f;
        glm::vec3 er = quatErrorAxisAngleWorld(glm::normalize(fk.rot), qTargetN, &angErr0);

        if (posErr0 <= posTolerance && angErr0 <= rotToleranceRad) return true;

        // Build full 6D Jacobian columns
        glm::vec3 pEE = fk.pos;
        for (int k = 0; k < n; ++k) {
            const URDFJoint& j = urdf.Joints()[chain.jointIdx[k]];

            glm::vec3 pJ    = glm::vec3(jointFrames[k] * glm::vec4(0,0,0,1));
            glm::vec3 axisW = glm::normalize(glm::vec3(jointFrames[k] * glm::vec4(j.axis, 0.0f)));
            if (glm::length(axisW) < 1e-6f) axisW = glm::vec3(0,0,0);

            Jcols[k].v = glm::cross(axisW, (pEE - pJ));
            Jcols[k].w = axisW;
        }

        // Ramp rotation weight when far away in position
        float ramp = 0.0f;
        {
            float far = std::max(posTolerance * 10.0f, posTolerance + 1e-6f);
            float t = (far - posErr0) / (far - posTolerance);
            ramp = clampf(t, 0.0f, 1.0f);
            ramp = ramp * ramp * (3.0f - 2.0f * ramp);
        }
        float rw = std::max(0.0f, rotWeight) * ramp;

        // Solve coupled 6D DLS
        if (!solveTaskDLS6(Jcols, ep, er, lambdaLocal, rw, dq)) return false;

        // Clamp step globally
        float maxAbs = 0.0f;
        for (int i = 0; i < n; ++i) maxAbs = std::max(maxAbs, std::fabs(dq[i]));
        if (maxAbs > maxStepRad && maxAbs > 1e-12f) {
            float s = maxStepRad / maxAbs;
            for (int i = 0; i < n; ++i) dq[i] *= s;
        }

        // Line search: accept if combined metric improves
        std::vector<float> snap;
        snapshotChainAngles(urdf, chain, snap);

        float metric0 = posErr0 * posErr0 + (rw * angErr0) * (rw * angErr0);

        bool accepted = false;
        float alpha = 1.0f;

        for (int ls = 0; ls < 6; ++ls) {
            for (int k = 0; k < n; ++k) {
                const URDFJoint& j = urdf.Joints()[chain.jointIdx[k]];
                if (!(j.type == JointType::Revolute || j.type == JointType::Continuous)) continue;

                float q = urdf.GetJointAngle(j.name);
                q += alpha * dq[k];

                if (j.type == JointType::Revolute && j.hasLimits) {
                    q = clampf(q, j.lower, j.upper);
                }
                urdf.SetJointAngle(j.name, q);
            }

            FKResult fk1 = ComputeFK(urdf, chain, jointFrames);
            float posErr1 = glm::length(targetPosWorld - fk1.pos);

            float angErr1 = 0.0f;
            (void)quatErrorAxisAngleWorld(glm::normalize(fk1.rot), qTargetN, &angErr1);

            float metric1 = posErr1 * posErr1 + (rw * angErr1) * (rw * angErr1);

            if (metric1 < metric0) {
                accepted = true;
                break;
            }

            restoreChainAngles(urdf, chain, snap);
            alpha *= 0.5f;
        }

        if (!accepted) {
            lambdaLocal = std::min(lambdaLocal * 2.0f, 50.0f);
        } else {
            lambdaLocal = std::max(lambdaLocal * 0.7f, 1e-6f);
        }
    }

    return false;
}

} // namespace URDFIK
