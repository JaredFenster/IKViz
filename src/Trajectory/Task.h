#pragma once
#ifndef ARMVIZ_TASK_H
#define ARMVIZ_TASK_H

#endif //ARMVIZ_TASK_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include "../Objects/Cube.h"
#include "Robot/RobotScene.h"
#include "Robot/URDFRobot.h"
#include "ImGuizmo/ImGuizmo.h"
#include "InverseKinematics/IK.h"
#include "Trajectory/Trajectory.h"
#include "Trajectory/Jog.h"
#include "Utilities/Utilities.h"
#include "Objects/Cube.h"
#include "Trajectory/Task.h"





class Task{

struct move
{
    glm::vec3 pos;
    glm::quat quat;

    bool gripState;
    Cube *Target;
};

public:

    Task(RobotScene *r){ robot = r; }
    void addTask(move);
    RobotScene* robot;

private:
    std::vector<move> tasks;
};