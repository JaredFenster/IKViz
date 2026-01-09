#pragma once
#ifndef ARMVIZ_TASK_H
#define ARMVIZ_TASK_H

#endif //ARMVIZ_TASK_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include "../Objects/Cube.h"






class Task{

struct move
{
    glm::vec3 pos;
    glm::quat quat;

    bool gripState;
    Cube *Target;
};


public:
    void addTask(move);

private:
    std::vector<move> tasks;
};