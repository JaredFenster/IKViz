#include "Trajectory.h"
#include <glm/gtx/quaternion.hpp> // for slerp

void Trajectory::GeneratePoints(const glm::vec3& currentPos,
                                const glm::vec3& newPos,
                                int density)
{
    points.clear();
    density = std::max(2, density);

    for (int k = 0; k < density; ++k) {
        float t = (density == 1) ? 1.0f : (float)k / (float)(density - 1);
        points.push_back(glm::mix(currentPos, newPos, t));
    }
}

void Trajectory::GeneratePoses(const glm::vec3& startPos, const glm::quat& startRot,
                               const glm::vec3& endPos,   const glm::quat& endRot,
                               int density)
{
    poses.clear();
    points.clear(); // optional: keep points in sync for debugging/line drawing

    density = std::max(2, density);

    glm::quat q0 = glm::normalize(startRot);
    glm::quat q1 = glm::normalize(endRot);

    // ensure shortest path for slerp
    if (glm::dot(q0, q1) < 0.0f) q1 = -q1;

    poses.reserve(density);
    points.reserve(density);

    for (int k = 0; k + 1 < density; ++k) {
        float t = (float)k / (float)(density - 1);

        PoseSample s;
        s.pos = glm::mix(startPos, endPos, t);
        s.rot = glm::normalize(glm::slerp(q0, q1, t));

        poses.push_back(s);
        points.push_back(s.pos);
    }
    PoseSample last;
    last.pos = endPos;
    last.rot = endRot;
    poses.push_back(last);
    points.push_back(last.pos);
}
