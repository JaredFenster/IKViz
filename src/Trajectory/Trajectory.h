#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>

class Trajectory {
public:
    // existing
    void GeneratePoints(const glm::vec3& currentPos, const glm::vec3& newPos, int density);

    // NEW: generate both position + rotation
    void GeneratePoses(const glm::vec3& startPos, const glm::quat& startRot,
                       const glm::vec3& endPos,   const glm::quat& endRot,
                       int density);

    glm::vec3 getPoint(int index) const { return points.at(index); }
    glm::vec3 getPos(int index)   const { return poses.at(index).pos; }
    glm::quat getRot(int index)   const { return poses.at(index).rot; }

    int getNumPoints() const { return (int)points.size(); }
    int getNumPoses()  const { return (int)poses.size(); }

private:
    struct PoseSample {
        glm::vec3 pos;
        glm::quat rot;
    };

    std::vector<glm::vec3> points;
    std::vector<PoseSample> poses;
};
