//
// Created by jared on 2026-01-07.
//
#pragma once
#ifndef ARMVIZ_JOG_H
#define ARMVIZ_JOG_H

#endif //ARMVIZ_JOG_H

#include "../Robot/RobotScene.h"
#include "../Robot/URDFRobot.h"
#include "../ImGuizmo/ImGuizmo.h"
#include "../InverseKinematics/IK.h"
#include "../Trajectory/Trajectory.h"

class Jog
{
    public:

    void startJog(bool jogInterpRotation,
        Trajectory *traj,
        RobotScene *robot,
        bool ikUseOrientation,
        float ikPosTol,
        float ikRotTol,
        float ikRotWeight,
        int jogStride,
        float ikMaxStepDeg,
        float ikLambda,
        int ikMaxIter,
        URDFIK::ChainInfo *chain,
        glm::quat jogEndRot);

        bool getJoggingStatus() const { return jogging; }
        void restartJogging() { jogging = true; jogIndex = 0; }
        void stopJogging() { jogging = false; jogIndex = 0;}
        int getJogIndex() const { return jogIndex; };
    private:
        int jogIndex = 0;
        bool jogging = false;
};