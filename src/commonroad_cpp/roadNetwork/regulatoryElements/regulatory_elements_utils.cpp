//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <algorithm>

#include "../lanelet/lane.h"

#include "../../predicates/predicate_config.h"
#include "../intersection/intersection_operations.h"
#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"
#include "regulatory_elements_utils.h"

std::set<std::shared_ptr<TrafficLight>>
regulatory_elements_utils::activeTrafficLights(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                               const std::shared_ptr<RoadNetwork> &roadNetwork) {
    std::set<std::shared_ptr<TrafficLight>> trafficLights;
    auto lanelets{obs->getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    for (const auto &lanelet : lanelets) {
        for (const auto &light : lanelet->getTrafficLights()) {
            if (light->isActive() and light->getElementAtTime(timeStep).color != TrafficLightState::inactive)
                trafficLights.insert(light);
        }
    }
    return trafficLights;
}

bool regulatory_elements_utils::atTrafficLightDirState(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                                  const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                  TurningDirection turnDir, TrafficLightState tlState) {
    if (!intersection_operations::onIncoming(timeStep, obs, roadNetwork))
        return false;
    std::unordered_set<TurningDirection> relevantTrafficLightDirections;
    switch (turnDir) {
    case TurningDirection::left:
        relevantTrafficLightDirections = {TurningDirection::left, TurningDirection::leftStraight,
                                          TurningDirection::leftRight, TurningDirection::all};
        break;
    case TurningDirection::right:
        relevantTrafficLightDirections = {TurningDirection::leftRight, TurningDirection::right,
                                          TurningDirection::straightRight, TurningDirection::all};
        break;
    case TurningDirection::straight:
        relevantTrafficLightDirections = {TurningDirection::straight, TurningDirection::straightRight,
                                          TurningDirection::leftStraight, TurningDirection::all};
        break;
    case TurningDirection::all:
        relevantTrafficLightDirections = {TurningDirection::left,      TurningDirection::leftStraight,
                                          TurningDirection::leftRight, TurningDirection::all,
                                          TurningDirection::right,     TurningDirection::straightRight,
                                          TurningDirection::straight};
        break;
    default:
        relevantTrafficLightDirections = {TurningDirection::left,      TurningDirection::leftStraight,
                                          TurningDirection::leftRight, TurningDirection::all,
                                          TurningDirection::right,     TurningDirection::straightRight,
                                          TurningDirection::straight};
    }
    auto activeTl{activeTrafficLights(timeStep, obs, roadNetwork)};
    for (const auto &light : activeTl) {
        if (std::any_of(relevantTrafficLightDirections.begin(), relevantTrafficLightDirections.end(),
                        [light](const TurningDirection &relevantDirection) {
                            return relevantDirection == light->getDirection();
                        }) and
            light->getElementAtTime(timeStep).color == tlState)
            return true;
    }
    return false;
}

double regulatory_elements_utils::speedLimit(const std::shared_ptr<Lanelet> &lanelet, const std::string &signId) {
    double limit = PredicateParameters().maxPositiveDouble;
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = lanelet->getTrafficSigns();
    for (const std::shared_ptr<TrafficSign> &signPtr : trafficSigns) {
        for (const std::shared_ptr<TrafficSignElement> &elemPtr : signPtr->getTrafficSignElements()) {
            if (elemPtr->getId() == signId) {
                double signLimit = std::stod(elemPtr->getAdditionalValues()[0]);
                if (limit > signLimit)
                    limit = signLimit;
            }
        }
    }
    return limit;
}

double regulatory_elements_utils::speedLimit(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                             const std::string &signId) {
    std::vector<double> speedLimits{std::numeric_limits<double>::max()};
    for (const auto &lanelet : lanelets) {
        speedLimits.push_back(speedLimit(lanelet, signId));
    }
    return *std::min_element(speedLimits.begin(), speedLimits.end());
}

double regulatory_elements_utils::speedLimitSuggested(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                                      const std::string &signId) {
    double vMaxLane{speedLimit(lanelets, signId)};
    if (vMaxLane == PredicateParameters().maxPositiveDouble)
        return PredicateParameters().desiredInterstateVelocity;
    else
        return std::min(PredicateParameters().desiredInterstateVelocity, vMaxLane);
}

double regulatory_elements_utils::requiredVelocity(const std::shared_ptr<Lanelet> &lanelet, const std::string &signId) {
    double limit = 0;
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = lanelet->getTrafficSigns();
    for (const std::shared_ptr<TrafficSign> &signPtr : trafficSigns) {
        for (const std::shared_ptr<TrafficSignElement> &elemPtr : signPtr->getTrafficSignElements()) {
            if (elemPtr->getId() == signId) {
                double signLimit = std::stod(elemPtr->getAdditionalValues()[0]);
                if (limit < signLimit)
                    limit = signLimit;
            }
        }
    }
    return limit;
}

double regulatory_elements_utils::requiredVelocity(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                                   const std::string &signId) {
    std::vector<double> speedLimits{0.0};
    for (const auto &lanelet : lanelets) {
        speedLimits.push_back(requiredVelocity(lanelet, signId));
    }
    return *std::max_element(speedLimits.begin(), speedLimits.end());
}

double regulatory_elements_utils::typeSpeedLimit(ObstacleType obstacleType) {
    switch (obstacleType) {
    case ObstacleType::truck:
        return 22.22;
    default:
        return std::numeric_limits<double>::max();
    }
}
