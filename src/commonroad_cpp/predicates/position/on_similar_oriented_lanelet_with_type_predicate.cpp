//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/roadNetwork/road_network.h"
#include "on_similar_oriented_lanelet_with_type_predicate.h"

bool OnSimilarOrientedLaneletWithTypePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &let : obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)) {
        auto lane{world->getRoadNetwork()
                      ->findLanesByContainedLanelet(let->getId())
                      .at(0)}; // just use one lane since all lanes are generated based on single lanelet
        auto orientationDif{obstacleK->getCurvilinearOrientation(timeStep, lane) -
                            lane->getOrientationAtPosition(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                                           obstacleK->getStateByTimeStep(timeStep)->getYPosition())};
        if (abs(orientationDif) < 0.3)
            lanelets.push_back(let);
    }
    return std::any_of(lanelets.begin(), lanelets.end(),
                       [additionalFunctionParameters](const std::shared_ptr<Lanelet> &lanelet) {
                           return lanelet->hasLaneletType(additionalFunctionParameters->laneletType);
                       });
}

double OnSimilarOrientedLaneletWithTypePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("OnLaneletWithTypePredicate does not support robust evaluation!");
}

Constraint OnSimilarOrientedLaneletWithTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("OnLaneletWithTypePredicate does not support constraint evaluation!");
}
OnSimilarOrientedLaneletWithTypePredicate::OnSimilarOrientedLaneletWithTypePredicate() : CommonRoadPredicate(false) {}
