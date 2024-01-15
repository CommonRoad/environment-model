//
// Created by Sebastian Maierhofer and Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/position/left_of_predicate.h>

bool LeftOfPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Obstacle>> allObs{world->getObstacles()};
    allObs.emplace_back(obstacleK);
    auto leftObstacles{obstacle_operations::obstaclesLeft(timeStep, allObs, obstacleP, world->getRoadNetwork())};
    return std::any_of(leftObstacles.begin(), leftObstacles.end(), [obstacleK](const std::shared_ptr<Obstacle> &obs) {
        return obstacleK->getId() == obs->getId();
    });
}

double
LeftOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                  const std::shared_ptr<Obstacle> &obstacleK,
                                  const std::shared_ptr<Obstacle> &obstacleP,
                                  const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Left Of does not support robust evaluation!");
}

Constraint LeftOfPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Left Of does not support constraint evaluation!");
}

LeftOfPredicate::LeftOfPredicate() : CommonRoadPredicate(true) {}
