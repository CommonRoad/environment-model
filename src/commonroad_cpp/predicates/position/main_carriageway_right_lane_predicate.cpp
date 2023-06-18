//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/main_carriageway_right_lane_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/world.h>

bool MainCarriagewayRightLanePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep);
    return std::any_of(lanelets.begin(), lanelets.end(), [](const std::shared_ptr<Lanelet> &lanelet) {
        return (lanelet->getAdjacentRight().adj == nullptr or
                ((lanelet->getAdjacentRight().adj != nullptr) and
                 (!lanelet->getAdjacentRight().oppositeDir) and
                 !lanelet->getAdjacentRight().adj->hasLaneletType(LaneletType::mainCarriageWay))) and
               lanelet->hasLaneletType(LaneletType::mainCarriageWay);
    });
}

double MainCarriagewayRightLanePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("In Rightmost Lane Predicate does not support robust evaluation!");
}

Constraint MainCarriagewayRightLanePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("In Rightmost Lane Predicate does not support constraint evaluation!");
}

MainCarriagewayRightLanePredicate::MainCarriagewayRightLanePredicate() : CommonRoadPredicate(false) {}
