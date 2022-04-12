//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "../../roadNetwork/lanelet/lane.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/world.h>

#include "../../roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include "relevant_traffic_light_predicate.h"

bool RelevantTrafficLightPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    if (!regulatory_elements_utils::activeTrafficLights(timeStep, obstacleK, world->getRoadNetwork()).empty())
        return true;
    else {
        for (const auto &lanelet : obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)) {
            auto lanes{world->getRoadNetwork()->findLanesByBaseLanelet(lanelet->getId())};
            for (const auto &lane : lanes) {
                auto relevantLanelets{lane->getSuccessorLanelets(lanelet)};
                for (const auto &let : relevantLanelets) {
                    for (const auto &light : let->getTrafficLights())
                        if (light->isActive() or light->getElementAtTime(timeStep).color != TrafficLightState::inactive)
                            return true;
                }
            }
        }
    }
    return false;
}

double RelevantTrafficLightPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("RelevantTrafficLightPredicate does not support robust evaluation!");
}
Constraint RelevantTrafficLightPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("RelevantTrafficLightPredicate does not support constraint evaluation!");
}
RelevantTrafficLightPredicate::RelevantTrafficLightPredicate() : CommonRoadPredicate(false) {}
