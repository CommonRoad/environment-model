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

#include <commonroad_cpp/predicates/position/approach_uncontrolled_intersection_predicate.h>

bool ApproachUncontrolledIntersectionPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    ApproachIntersectionPredicate approachIntersection = ApproachIntersectionPredicate();
    if (!approachIntersection.booleanEvaluation(timeStep, world, obstacleK, obstacleP)) {
        return false;
    }
    auto currentIntersection = intersection_operations::currentIntersection(timeStep, world, obstacleK);
    for (auto &incoming : currentIntersection->getIncomingGroups()) {
        for (auto &lanelet : incoming->getIncomingLanelets()) {
            if (!lanelet->getTrafficLights().empty() || !lanelet->getTrafficSigns().empty() ||
                !(lanelet->getStopLine() == nullptr)) {
                return false;
            }
        }
    }
    return true;
}
double ApproachUncontrolledIntersectionPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("ApproachUncontrolledIntersectionPredicate does not support robust evaluation!");
}
Constraint ApproachUncontrolledIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("ApproachUncontrolledIntersectionPredicate does not support constraint evaluation!");
}
ApproachUncontrolledIntersectionPredicate::ApproachUncontrolledIntersectionPredicate() : CommonRoadPredicate(false) {}
