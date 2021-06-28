//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/world.h>

#include "at_red_traffic_light_predicate.h"
#include "../../roadNetwork/regulatoryElements/regulatory_elements_utils.h"

bool AtRedTrafficLightPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleK,
                                                   const std::shared_ptr<Obstacle> &obstacleP) {
    return regulatory_elements_utils::atRedTrafficLight(timeStep, obstacleK, world->getRoadNetwork(),
                                                        TurningDirections::all);
}
double AtRedTrafficLightPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("AtRedTrafficLightPredicate does not support robust evaluation!");
}
Constraint AtRedTrafficLightPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                            const std::shared_ptr<Obstacle> &obstacleK,
                                                            const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("AtRedTrafficLightPredicate does not support constraint evaluation!");
}
