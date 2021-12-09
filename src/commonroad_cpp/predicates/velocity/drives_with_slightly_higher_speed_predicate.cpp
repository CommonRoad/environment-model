//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "drives_with_slightly_higher_speed_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include "commonroad_cpp/predicates/velocity/in_standstill_predicate.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>
bool DrivesWithSlightlyHigherSpeedPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                               const std::shared_ptr<Obstacle> &obstacleK,
                                                               const std::shared_ptr<Obstacle> &obstacleP) {
    return 0 < (obstacleK->getStateByTimeStep(timeStep)->getVelocity() -
                obstacleP->getStateByTimeStep(timeStep)->getVelocity()) < parameters.slightlyHigherSpeedDifference;
}

Constraint DrivesWithSlightlyHigherSpeedPredicate::constraintEvaluation(size_t timeStep,
                                                                        const std::shared_ptr<World> &world,
                                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                                        const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Drives With Slightly Higher Speed Predicate does not support constraint evaluation!");
}

double DrivesWithSlightlyHigherSpeedPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                                const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Drives With Slightly Higher Speed Predicate does not support robust evaluation!");
}