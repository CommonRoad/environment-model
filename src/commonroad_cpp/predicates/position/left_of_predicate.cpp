//
// Created by Sebastian Maierhofer and Evald Nexhipi.
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
    auto referenceK{obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep, world->getIdCounterRef())};
    if (obstacleK->rightD(timeStep) >= obstacleP->leftD(timeStep, referenceK))
        return false;
    else {
        if (obstacleK->rearS(timeStep) <= obstacleP->frontS(timeStep, referenceK) and
            obstacleP->frontS(timeStep, referenceK) <= obstacleK->frontS(timeStep))
            return true;
        if (obstacleK->rearS(timeStep) < obstacleP->rearS(timeStep, referenceK) and
            obstacleP->rearS(timeStep, referenceK) < obstacleK->frontS(timeStep))
            return true;
        return (obstacleP->rearS(timeStep, referenceK) < obstacleK->rearS(timeStep)) and
               (obstacleK->frontS(timeStep) < obstacleP->frontS(timeStep, referenceK));
    }
}

double LeftOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                         const std::shared_ptr<Obstacle> &obstacleK,
                                         const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Left Of does not support robust evaluation!");
}

Constraint LeftOfPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Left Of does not support constraint evaluation!");
}

LeftOfPredicate::LeftOfPredicate() : CommonRoadPredicate(true) {}
