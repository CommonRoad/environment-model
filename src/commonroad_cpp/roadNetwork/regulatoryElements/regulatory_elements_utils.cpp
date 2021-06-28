//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <algorithm>

#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>

#include "regulatory_elements_utils.h"
#include "../intersection/intersection_operations.h"
#include "commonroad_cpp/auxiliaryDefs/traffic_signs.h"

std::set<std::shared_ptr<TrafficLight>>
regulatory_elements_utils::activeTrafficLights(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                               const std::shared_ptr<RoadNetwork> &roadNetwork) {
    std::set<std::shared_ptr<TrafficLight>> trafficLights;
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &la : lanelets) {
        for (const auto &tl : la->getTrafficLights()) {
            if (tl->isActive() and tl->getElementAtTime(timeStep).color != TrafficLightState::inactive)
                trafficLights.insert(tl);
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
    for (const auto &tl : activeTl) {
        if (std::any_of(relevantTrafficLightDirections.begin(), relevantTrafficLightDirections.end(),
                        [tl](const TurningDirections &relevantDirection) {
                            return relevantDirection == tl->getDirection();
                        }) and
            tl->getElementAtTime(timeStep).color == TrafficLightState::red)
            return true;
    }
    return false;
}

bool regulatory_elements_utils::trafficSignReferencesStopSign(std::shared_ptr<TrafficSign> sign,
                                                              SupportedTrafficSignCountry country) {
    const auto signId{TrafficSignLookupTableByCountry.at(country)->at(TrafficSignTypes::STOP)};
    auto elements{sign->getTrafficSignElements()};
    return std::any_of(elements.begin(), elements.end(),
                       [signId](std::shared_ptr<TrafficSignElement> elem) { return elem->getId() == signId; });
}