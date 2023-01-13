//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/braking/braking_with_acceleration_possible_at_intersection_predicate.h>

bool BrakingWithAccelerationPossibleAtIntersectionPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    double distanceIntersection{0.0};

    double brakingDistance{std::pow(obstacleK->getStateByTimeStep(timeStep)->getVelocity(), 2) /
                           (2 * abs(parameters.intersectionBrakingPossible))};
    for (const auto &lane : lanelet_operations::createLanesBySingleLanelets(
             obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep), world->getRoadNetwork())) {
        for (const auto &letLane : lane->getContainedLanelets()) {
            if (letLane->hasLaneletType(LaneletType::incoming)) {
                auto curvPos{lane->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(
                    letLane->getCenterVertices().back().x, letLane->getCenterVertices().back().y)};
                auto ownPos{lane->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(
                    obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                    obstacleK->getStateByTimeStep(timeStep)->getYPosition())};
                distanceIntersection = curvPos.x() - ownPos.x();
                break;
            }
        }
        if (distanceIntersection != 0.0)
            break;
    }

    return brakingDistance < distanceIntersection;
}

Constraint BrakingWithAccelerationPossibleAtIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("BrakingWithAccelerationPossible does not support robust evaluation!");
}

double BrakingWithAccelerationPossibleAtIntersectionPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("BrakingWithAccelerationPossible does not support robust evaluation!");
}

BrakingWithAccelerationPossibleAtIntersectionPredicate::BrakingWithAccelerationPossibleAtIntersectionPredicate()
    : CommonRoadPredicate(false) {}