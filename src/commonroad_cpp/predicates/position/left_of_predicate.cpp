//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>
#include <commonroad_cpp/world.h>

#include "left_of_predicate.h"

bool LeftOfPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP) {

    if (obstacleK->leftD(timeStep) >= obstacleP->rightD(timeStep))
        return false;
    else{
        if (obstacleK->rearS(timeStep) <= obstacleP->frontS(timeStep) <= obstacleK->frontS(timeStep))
            return true;
        if (obstacleK->rearS(timeStep) < obstacleP->rearS(timeStep) < obstacleK->frontS(timeStep))
            return true;
        if ((obstacleP->rearS(timeStep) < obstacleK->rearS(timeStep)) and (obstacleK->frontS(timeStep) < obstacleP->frontS(timeStep)))
            return true;
        else
            return false;
    }
}

double LeftOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("PassesStopLinePredicate does not support robust evaluation!");
}

Constraint LeftOfPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("PassesStopLinePredicate does not support constraint evaluation!");
}
