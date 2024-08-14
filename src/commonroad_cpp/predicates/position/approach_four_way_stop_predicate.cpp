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

#include <commonroad_cpp/predicates/position/approach_four_way_stop_predicate.h>

bool ApproachFourWayStopPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters) {
    ApproachIntersectionPredicate approachIntersection = ApproachIntersectionPredicate();
    if (!approachIntersection.booleanEvaluation(timeStep, world, obstacleK, obstacleP)) {
        return false;
    }
    auto currentIntersection = intersection_operations::currentIntersection(timeStep, world, obstacleK);
    if (currentIntersection == nullptr || currentIntersection->getIncomingGroups().size() != 4) {
        return false;
    }
    // get current Intersection
    // look at all incomings of this intersection then looka at all lanelets from each incoming
    // if each lanelet has STOP sign it is 4 way stop
    for (const auto &incoming : currentIntersection->getIncomingGroups()) {
        for (const auto &lanelet : incoming->getIncomingLanelets()) {
            if (lanelet->hasTrafficSign(TrafficSignTypes::STOP_4_WAY)) {
                return true;
            }
        }
    }
    return false;
}
double ApproachFourWayStopPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP,
                                                      const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("Approach4WayStopPredicate does not support robust evaluation!");
}
Constraint ApproachFourWayStopPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("Approach4WayStopPredicate does not support constraint evaluation!");
}
ApproachFourWayStopPredicate::ApproachFourWayStopPredicate() : CommonRoadPredicate(false) {}
