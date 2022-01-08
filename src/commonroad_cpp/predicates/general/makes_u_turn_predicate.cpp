//
// Created by Sebastian Maierhofer and Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "makes_u_turn_predicate.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

bool MakesUTurnPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP) {
    std::vector<std::shared_ptr<Lane>> lanes = world->getRoadNetwork()->findLanesByContainedLanelet(
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)[0]->getId());
    return std::any_of(
        lanes.begin(), lanes.end(), [this, timeStep, obstacleK, world](const std::shared_ptr<Lane> &lane) {
            return parameters.uTurn <=
                   abs(obstacleK->getCurvilinearOrientation(world->getRoadNetwork(), timeStep) -
                       lane->getOrientationAtPosition(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                                      obstacleK->getStateByTimeStep(timeStep)->getYPosition()));
        });
}

Constraint MakesUTurnPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Makes U Turn Predicate does not support constraint evaluation!");
}

double MakesUTurnPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Makes U Turn Predicate does not support robust evaluation!");
}

MakesUTurnPredicate::MakesUTurnPredicate() : CommonRoadPredicate(false) {}
