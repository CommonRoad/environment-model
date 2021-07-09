//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "succeeds_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include "../../geometry/geometric_operations.h"

bool SucceedsPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                          const std::shared_ptr<Obstacle> &obstacleP,
                                          const std::shared_ptr<Obstacle> &obstacleK) {
    InFrontOfPredicate inFrontOfPredicate;
    InSameLanePredicate inSameLanePredicate;
    auto curPointOrientation{obstacleP->getReferenceLane()->getOrientationAtPosition(
        obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
        obstacleK->getStateByTimeStep(timeStep)->getYPosition())};
    return inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleP, obstacleK) and
        abs(geometric_operations::subtractOrientations(curPointOrientation, obstacleK->getStateByTimeStep(timeStep)->getGlobalOrientation())) <
            parameters.laneMatchingOrientation
           and
           inFrontOfPredicate.booleanEvaluation(
               timeStep, world, obstacleP,
               obstacleK); // TODO update orientation part since orientation representation is two-folded)
}

Constraint SucceedsPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleP,
                                                   const std::shared_ptr<Obstacle> &obstacleK) {
    throw std::runtime_error("StopLineInFrontPredicate does not support constraint evaluation!");
}

double SucceedsPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                           const std::shared_ptr<Obstacle> &obstacleP,
                                           const std::shared_ptr<Obstacle> &obstacleK) {
    throw std::runtime_error("StopLineInFrontPredicate does not support robust evaluation!");
}
