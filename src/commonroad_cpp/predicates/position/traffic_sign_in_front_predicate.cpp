//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "traffic_sign_in_front_predicate.h"
#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lane.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <Eigen/Core>
#include <geometry/curvilinear_coordinate_system.h>

bool TrafficSignInFrontPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    auto lanelets{obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)};
    const auto signId{TrafficSignLookupTableByCountry.at(world->getRoadNetwork()->getCountry())
                          ->at(additionalFunctionParameters->signType)};
    for (const auto &lanelet : lanelets) {
        for (const auto &sign : lanelet->getTrafficSigns()) {
            if (sign->getTrafficSignElementsOfType(signId).empty())
                return false;
            Eigen::Vector2d signPos{obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)
                                        ->getCurvilinearCoordinateSystem()
                                        ->convertToCurvilinearCoords(sign->getPosition().x, sign->getPosition().y)};

            if (obstacleK->frontS(world->getRoadNetwork(), timeStep) < signPos.x())
                return true;
        }
    }
    return false;
}

double TrafficSignInFrontPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("TrafficSignInFrontPredicate does not support robust evaluation!");
}

Constraint TrafficSignInFrontPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("TrafficSignInFrontPredicate does not support constraint evaluation!");
}
TrafficSignInFrontPredicate::TrafficSignInFrontPredicate() : CommonRoadPredicate(false) {}
