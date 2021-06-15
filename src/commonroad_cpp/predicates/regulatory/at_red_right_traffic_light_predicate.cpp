//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "at_red_right_traffic_light_predicate.h"
#include "../../roadNetwork/regulatoryElements/regulatory_elements_utils.h"

bool AtRedRightTrafficLightPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP) {
    return regulatory_elements_utils::atRedTrafficLight(timeStep, obstacleK, world->getRoadNetwork(),
                                                        TurningDirections::right);
}
double AtRedRightTrafficLightPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("AtRedRightTrafficLight does not support robust evaluation!");
}
Constraint AtRedRightTrafficLightPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("AtRedRightTrafficLight does not support constraint evaluation!");
}
