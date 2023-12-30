//
// Created by Sebastian Maierhofer and Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h"
#include "commonroad_cpp/world.h"
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>

#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/predicates/position/drives_leftmost_predicate.h>

bool DrivesLeftmostPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets =
        obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep);
    std::shared_ptr<Obstacle> vehicle_directly_left =
        obstacle_operations::obstacleDirectlyLeft(timeStep, world->getObstacles(), obstacleK, world->getRoadNetwork());

    if (vehicle_directly_left != nullptr and
        (vehicle_directly_left->rightD(timeStep, obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)) -
         obstacleK->leftD(world->getRoadNetwork(), timeStep)) < parameters.paramMap["closeToOtherVehicle"]) {
        return true;
    } else {
        std::vector<std::shared_ptr<Lane>> lanes{obstacleK->getOccupiedLanes(world->getRoadNetwork(), timeStep)};
        return std::all_of(lanes.begin(), lanes.end(), [obstacleK, this, timeStep](const std::shared_ptr<Lane> &lane) {
            return 0.5 * lane->getWidth(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                        obstacleK->getStateByTimeStep(timeStep)->getYPosition()) -
                       obstacleK->leftD(timeStep, lane) <=
                   parameters.paramMap["closeToLaneBorder"];
        });
    }
}

double DrivesLeftmostPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Drives Leftmost Predicate does not support robust evaluation!");
}

Constraint DrivesLeftmostPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Drives Leftmost Predicate does not support constraint evaluation!");
}

DrivesLeftmostPredicate::DrivesLeftmostPredicate() : CommonRoadPredicate(false) {}