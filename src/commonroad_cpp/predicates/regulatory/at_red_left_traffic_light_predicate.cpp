//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/world.h>

#include "../../roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include "at_red_left_traffic_light_predicate.h"

bool AtRedLeftTrafficLightPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP) {
    return regulatory_elements_utils::atRedTrafficLight(timeStep, obstacleK, world->getRoadNetwork(),
                                                        TurningDirections::left);
}
double AtRedLeftTrafficLightPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("AtRedLeftTrafficLightPredicate does not support robust evaluation!");
}
Constraint AtRedLeftTrafficLightPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                                const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("AtRedLeftTrafficLightPredicate does not support constraint evaluation!");
}
