#include "Trajectory.h"
#include <glm/glm.hpp>
#include <glad/glad.h>
#include <iostream>

void Trajectory::GeneratePoints(const glm::vec3& currentPos, const glm::vec3& newPos, int density){
    points.clear();

    density = std::max(2, density); // at least start+end

    for (int k = 0; k < density; ++k) {
        float t = static_cast<float>(k) / static_cast<float>(density - 1); // [0..1]
        glm::vec3 p = glm::mix(currentPos, newPos, t);
        points.push_back(p);
    }
}
