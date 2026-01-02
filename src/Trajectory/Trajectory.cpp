#include "Trajectory.h"
#include <glm/glm.hpp>
#include <glad/glad.h>

std::vector<glm::vec3> Trajectory::GeneratePoints(glm::vec3 currentPos, glm::vec3 newPos, int density){

    std::vector<glm::vec3> points;

    // density in units points per unit
    glm::vec3 path = newPos - currentPos;
    glm::vec3 dir = glm::normalize(path);

    float dist = glm::length(path);
    float step = dist/density;

    for(float i = 0.0f; i <= dist; i += step){
        glm::vec3 p = currentPos + dir * i;
        points.push_back(p);
    }

}