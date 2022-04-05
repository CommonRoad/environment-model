//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <memory>

#include <geometry/curvilinear_coordinate_system.h>

#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>
#include <commonroad_cpp/world.h>

#include "at_stop_sign_predicate.h"

bool AtStopSignPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            OptionalPredicateParameters additionalFunctionParameters) {
    auto lanelets{obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)};
    for (const auto &let : lanelets) {
        auto signs{let->getTrafficSigns()};
        if (std::any_of(signs.begin(), signs.end(), [world](const std::shared_ptr<TrafficSign> &sign) {
                return regulatory_elements_utils::trafficSignReferencesStopSign(sign,
                                                                                world->getRoadNetwork()->getCountry());
            }))
            return true;
    }
    return false;
}

double AtStopSignPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP,
                                             OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("AtStopSignPredicate does not support robust evaluation!");
}

Constraint AtStopSignPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("AtStopSignPredicate does not support constraint evaluation!");
}
AtStopSignPredicate::AtStopSignPredicate() : CommonRoadPredicate(false) {}
