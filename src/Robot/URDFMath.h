#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

inline glm::mat4 urdfXYZRPY(const glm::vec3& xyz, const glm::vec3& rpy) {
    glm::mat4 T(1.0f);
    T = glm::translate(T, xyz);
    // rpy = (roll=x, pitch=y, yaw=z)
    T = glm::rotate(T, rpy.z, glm::vec3(0,0,1));
    T = glm::rotate(T, rpy.y, glm::vec3(0,1,0));
    T = glm::rotate(T, rpy.x, glm::vec3(1,0,0));
    return T;
}
