#pragma once


#ifndef ARMVIZ_DYNAMIC_H
#define ARMVIZ_DYNAMIC_H

#endif //ARMVIZ_DYNAMIC_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "../Robot/RobotScene.h"

class Cube
{
public:

    Cube(glm::vec3 position, glm::vec3 rotation, glm::vec3 scale);

    glm::vec3 position;
    glm::quat rotation;

    glm::vec3 pickVector; // robot snaps onto end of vector then follows to pick
    bool grabbed = false;

    float gravity = 9.8f;
    float groundPlaneOffset = 0.0f;


private:
    RobotScene *Parent;



};