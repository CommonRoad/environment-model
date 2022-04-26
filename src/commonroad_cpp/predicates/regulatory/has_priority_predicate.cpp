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
    std::shared_ptr<TrafficSignElement> tmpSign{std::make_shared<TrafficSignElement>("102")};
    for (const auto &tse : relevantTrafficSignElements) {
        if (!std::any_of(
                relevantTrafficSignElements.begin(), relevantTrafficSignElements.end(),
                [tse](const std::shared_ptr<TrafficSignElement> &tel) { return tel->getId() == tse->getId(); }))
            continue;
        if (tmpSign->getId() != TrafficSignIDGermany.at(TrafficSignTypes::WARNING_RIGHT_BEFORE_LEFT) and
            (tse->getId() == TrafficSignIDGermany.at(TrafficSignTypes::PRIORITY) or
             tse->getId() == TrafficSignIDGermany.at(TrafficSignTypes::YIELD)))
            continue;
        tmpSign = tse;
    }
    return tmpSign;
}

int extractPriorityTrafficSign(const std::vector<std::shared_ptr<Lanelet>> &lanelets, TurningDirection dir) {
    std::shared_ptr<TrafficSignElement> tmpSign;
    int currentPriorityValue{-123456789}; // todo use min int
    for (const auto &let : lanelets) {
        auto trs{extractPriorityTrafficSign(let)};
        if (priorityTable.count(trs->getId()) == 1 and (currentPriorityValue == -123456789 or
                                                        priorityTable.at(trs->getId()).at(static_cast<size_t>(dir)) < currentPriorityValue))
            currentPriorityValue = priorityTable.at(trs->getId()).at(static_cast<size_t>(dir));
    }
    return currentPriorityValue;
}

int getPriority(size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork, const std::shared_ptr<Obstacle> &obs,
                TurningDirection dir) {
    auto lanelets{obs->getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    std::vector<std::shared_ptr<Lanelet>> relevantIncomingLanelets;
    for (const auto &let : lanelets) {
        if (let->hasLaneletType(LaneletType::incoming))
            relevantIncomingLanelets.push_back(let);
    }
    return extractPriorityTrafficSign(relevantIncomingLanelets, dir);
}

bool HasPriorityPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    int prioK{getPriority(timeStep, world->getRoadNetwork(), obstacleK,
                          additionalFunctionParameters->turningDirection.at(0))};
    int prioP{getPriority(timeStep, world->getRoadNetwork(), obstacleP,
                          additionalFunctionParameters->turningDirection.at(1))};
    return prioK > prioP and prioK != -123456789 and prioP != -123456789;
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
