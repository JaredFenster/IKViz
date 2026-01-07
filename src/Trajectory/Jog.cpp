//
// Created by jared on 2026-01-07.
//

#include "Jog.h"

#include "../Robot/RobotScene.h"
#include "../Robot/URDFRobot.h"
#include "../ImGuizmo/ImGuizmo.h"
#include "../InverseKinematics/IK.h"
#include "../Trajectory/Trajectory.h"
#include <iostream>

void Jog::startJog(bool jogInterpRotation,
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
        glm::quat jogEndRot)

{

        if (jogInterpRotation)
        {
            const int n = traj->getNumPoses();
            if (n < 2)
            {
                jogging = false;
                jogIndex = 0;
            }
            else
            {
                jogging = true;
                jogIndex = std::clamp(jogIndex, 0, n - 1);

                glm::vec3 jogPos = traj->getPos(jogIndex);
                glm::quat jogRot = traj->getRot(jogIndex);

                float rotTolUse = ikUseOrientation ? ikRotTol : 999.0f;
                float rotWUse = ikUseOrientation ? ikRotWeight : 0.0f;

                (void)URDFIK::SolvePoseHierDLS(
                    robot->Robot(), *chain,
                    jogPos, jogRot,
                    ikMaxIter, ikPosTol, rotTolUse,
                    ikMaxStepDeg, rotWUse, ikLambda
                );

                jogIndex += std::max(1, jogStride);
                if (jogIndex >= n)
                {
                    jogIndex = 0;
                    jogging = false;
                }
            }
        }
        else
        {
            const int n = traj->getNumPoints();
            if (n < 2)
            {
                jogging = false;
                jogIndex = 0;
            }
            else
            {
                jogIndex = std::clamp(jogIndex, 0, n - 1);
                glm::vec3 jogPos = traj->getPoint(jogIndex);

                float rotTolUse = ikUseOrientation ? ikRotTol : 999.0f;
                float rotWUse = ikUseOrientation ? ikRotWeight : 0.0f;

                (void)URDFIK::SolvePoseHierDLS(
                    robot->Robot(), *chain,
                    jogPos, jogEndRot,
                    ikMaxIter, ikPosTol, rotTolUse,
                    ikMaxStepDeg, rotWUse, ikLambda
                );

                jogIndex += std::max(1, jogStride);
                if (jogIndex >= n)
                {
                    jogIndex = 0;
                    jogging = false;
                }
            }
        }
}