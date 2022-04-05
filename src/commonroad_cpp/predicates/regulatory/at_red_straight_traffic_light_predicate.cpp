//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/world.h>

#include "../../roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include "at_red_straight_traffic_light_predicate.h"

bool AtRedStraightTrafficLightPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                           const std::shared_ptr<Obstacle> &obstacleK,
                                                           const std::shared_ptr<Obstacle> &obstacleP,
                                                           OptionalPredicateParameters additionalFunctionParameters) {
    return regulatory_elements_utils::atRedTrafficLight(timeStep, obstacleK, world->getRoadNetwork(),
                                                        TurningDirections::straight);
}
double AtRedStraightTrafficLightPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                            const std::shared_ptr<Obstacle> &obstacleK,
                                                            const std::shared_ptr<Obstacle> &obstacleP,
                                                            OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("AtRedStraightTrafficLightPredicate does not support robust evaluation!");
}
Constraint AtRedStraightTrafficLightPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("AtRedStraightTrafficLightPredicate does not support constraint evaluation!");
}
AtRedStraightTrafficLightPredicate::AtRedStraightTrafficLightPredicate() : CommonRoadPredicate(false) {}
