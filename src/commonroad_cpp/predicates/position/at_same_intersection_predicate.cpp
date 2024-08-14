//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/approach_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/position/at_same_intersection_predicate.h>

bool AtSameIntersectionPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters) {

    auto currentIntersection_k = intersection_operations::currentIntersection(timeStep, world, obstacleK);
    auto currentIntersection_p = intersection_operations::currentIntersection(timeStep, world, obstacleP);
    if (currentIntersection_k != nullptr && currentIntersection_p != nullptr) {
        return currentIntersection_k->getId() == currentIntersection_p->getId();
    }

    return false;
}
double AtSameIntersectionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("AtSameIntersectionPredicate does not support robust evaluation!");
}
Constraint AtSameIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("AtSameIntersectionPredicate does not support constraint evaluation!");
}
AtSameIntersectionPredicate::AtSameIntersectionPredicate() : CommonRoadPredicate(true) {}
