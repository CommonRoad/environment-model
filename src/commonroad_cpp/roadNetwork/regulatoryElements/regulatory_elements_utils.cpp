//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <algorithm>

#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>

#include "../../predicates/predicate_config.h"
#include "../intersection/intersection_operations.h"
#include "commonroad_cpp/auxiliaryDefs/traffic_signs.h"
#include "regulatory_elements_utils.h"

std::set<std::shared_ptr<TrafficLight>>
regulatory_elements_utils::activeTrafficLights(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                               const std::shared_ptr<RoadNetwork> &roadNetwork) {
    std::set<std::shared_ptr<TrafficLight>> trafficLights;
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &lanelet : lanelets) {
        for (const auto &light : lanelet->getTrafficLights()) {
            if (light->isActive() and light->getElementAtTime(timeStep).color != TrafficLightState::inactive)
                trafficLights.insert(light);
        }
    }
    return trafficLights;
}

bool regulatory_elements_utils::atRedTrafficLight(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                                  const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                  TurningDirections turnDir) {
    if (!intersection_operations::onIncoming(timeStep, obs, roadNetwork))
        return false;
    std::vector<TurningDirections> relevantTrafficLightDirections;
    switch (turnDir) {
    case TurningDirections::left:
        relevantTrafficLightDirections = {TurningDirections::left, TurningDirections::leftStraight,
                                          TurningDirections::leftRight, TurningDirections::all};
        break;
    case TurningDirections::right:
        relevantTrafficLightDirections = {TurningDirections::leftRight, TurningDirections::right,
                                          TurningDirections::straightRight, TurningDirections::all};
        break;
    case TurningDirections::straight:
        relevantTrafficLightDirections = {TurningDirections::straight, TurningDirections::straightRight,
                                          TurningDirections::leftStraight, TurningDirections::all};
        break;
    case TurningDirections::all:
        relevantTrafficLightDirections = {TurningDirections::all};
        break;
    default:
        relevantTrafficLightDirections = {TurningDirections::all};
    }
    auto activeTl{activeTrafficLights(timeStep, obs, roadNetwork)};
    for (const auto &light : activeTl) {
        if (std::any_of(relevantTrafficLightDirections.begin(), relevantTrafficLightDirections.end(),
                        [light](const TurningDirections &relevantDirection) {
                            return relevantDirection == light->getDirection();
                        }) and
            light->getElementAtTime(timeStep).color == TrafficLightState::red)
            return true;
    }
    return false;
}

bool regulatory_elements_utils::trafficSignReferencesStopSign(const std::shared_ptr<TrafficSign> &sign,
                                                              SupportedTrafficSignCountry country) {
    const auto signId{TrafficSignLookupTableByCountry.at(country)->at(TrafficSignTypes::STOP)};
    auto elements{sign->getTrafficSignElements()};
    return std::any_of(elements.begin(), elements.end(),
                       [signId](const std::shared_ptr<TrafficSignElement> &elem) { return elem->getId() == signId; });
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
    std::vector<double> speedLimits;
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
    std::vector<double> speedLimits;
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