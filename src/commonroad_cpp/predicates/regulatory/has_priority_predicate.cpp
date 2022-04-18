//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <memory>

#include <geometry/curvilinear_coordinate_system.h>

#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"
#include "has_priority_predicate.h"

std::vector<std::shared_ptr<Lanelet>> incomingLaneletOfLanelet(const std::shared_ptr<Lanelet> &lanelet) {
    std::vector<std::shared_ptr<Lanelet>> incomingLanelets;
    if (lanelet->hasLaneletType(LaneletType::intersection)) {
        auto pre = lanelet->getPredecessors().at(0); // TODO currently only a single predecessor is considered
        while (!pre->hasLaneletType(LaneletType::incoming)) {
            pre = pre->getPredecessors().at(0);
        }
        incomingLanelets.push_back(pre);
    } else if (lanelet->hasLaneletType(LaneletType::incoming)) {
        incomingLanelets.push_back(lanelet);
    }
    return incomingLanelets;
}

std::vector<std::string> getRelevantPrioritySignIDs() {
    std::vector<std::string> keys;
    keys.reserve(priorityTable.size());
    for (const auto &[key, value] : priorityTable) {
        keys.push_back(key);
    }
    return keys;
}

std::shared_ptr<TrafficSignElement> extractPriorityTrafficSign(const std::shared_ptr<Lanelet> &lanelet) {
    static const std::vector<std::string> relevantPrioritySignIds{getRelevantPrioritySignIDs()};
    std::vector<std::shared_ptr<TrafficSignElement>> relevantTrafficSignElements;
    for (const auto &trs : lanelet->getTrafficSigns()) {
        auto trafficSignElements{trs->getTrafficSignElements()};
        relevantTrafficSignElements.insert(relevantTrafficSignElements.end(), trafficSignElements.begin(),
                                           trafficSignElements.end());
    }
    for (const auto &tse : relevantTrafficSignElements) {
        if (std::any_of(relevantTrafficSignElements.begin(), relevantTrafficSignElements.end(),
                        [tse](const std::shared_ptr<TrafficSignElement> &tel) { return tel->getId() == tse->getId(); }))
            continue;
        else
            return tse; // TODO don't just return first sign -> look at BA there it is different
    }
    return nullptr;
}

std::shared_ptr<TrafficSignElement> extractPriorityTrafficSign(const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    for (const auto &let : lanelets) { // TODO don't use just first value
        auto trs{extractPriorityTrafficSign(let)};
        if (trs == nullptr)
            continue;
        else
            return trs;
    }
    return nullptr;
}

int getPriority(size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork, const std::shared_ptr<Obstacle> &obs,
                TurningDirection dir) {
    auto lanelets{obs->getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    std::vector<std::shared_ptr<Lanelet>> relevantIncomingLanelets;
    for (const auto &let : lanelets) {
        auto incomingLanelets{incomingLaneletOfLanelet(let)};
        relevantIncomingLanelets.insert(relevantIncomingLanelets.end(), incomingLanelets.begin(),
                                        incomingLanelets.end());
    }
    auto prioTrafficSign{extractPriorityTrafficSign(relevantIncomingLanelets)};
    if (prioTrafficSign != nullptr)
        return priorityTable.at(prioTrafficSign->getId()).at(static_cast<size_t>(dir));
    else
        return priorityTable.at("102").at(static_cast<size_t>(dir));
}

bool HasPriorityPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return getPriority(timeStep, world->getRoadNetwork(), obstacleK,
                       additionalFunctionParameters->turningDirection.at(0)) >
           getPriority(timeStep, world->getRoadNetwork(), obstacleP,
                       additionalFunctionParameters->turningDirection.at(0));
}

double HasPriorityPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("HasPriorityPredicate does not support robust evaluation!");
}

Constraint HasPriorityPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("HasPriorityPredicate does not support constraint evaluation!");
}

HasPriorityPredicate::HasPriorityPredicate() : CommonRoadPredicate(true) {}
