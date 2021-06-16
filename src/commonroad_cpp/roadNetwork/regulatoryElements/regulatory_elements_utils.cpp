//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "regulatory_elements_utils.h"
#include "../intersection/intersection_operations.h"

std::set<std::shared_ptr<TrafficLight>>
regulatory_elements_utils::activeTrafficLights(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                               std::shared_ptr<RoadNetwork> roadNetwork) {
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
                                                  std::shared_ptr<RoadNetwork> roadNetwork, TurningDirections turnDir) {
    if (!intersection_operations::onIncoming(timeStep, obs, roadNetwork))
        return false;
    std::vector<TurningDirections> relevantTrafficLightDirections;
    switch (turnDir) {
    case TurningDirections::left:
        relevantTrafficLightDirections = {TurningDirections::left, TurningDirections::leftStraight,
                                          TurningDirections::leftRight};
        break;
    case TurningDirections::right:
        relevantTrafficLightDirections = {TurningDirections::leftRight, TurningDirections::straightRight,
                                          TurningDirections::straightRight};
        break;
    default:
        relevantTrafficLightDirections = {TurningDirections::straight, TurningDirections::leftStraight,
                                          TurningDirections::straightRight};
    }
    std::cout << "slfjksf" << '\n';
    auto activeTl{activeTrafficLights(timeStep, obs, roadNetwork)};
    for (const auto &tl : activeTl) {
      std::cout << tl.get();
        auto trafficLightState{tl->getElementAtTime(timeStep).color};
        if (std::any_of(relevantTrafficLightDirections.begin(), relevantTrafficLightDirections.end(),
                        [tl](const TurningDirections &relevantDirection) {
                            return relevantDirection == tl->getDirection();
                        }) and
            trafficLightState != TrafficLightState::green)
            return true;
    }
  std::cout << "asfsdf" << '\n';
    // use all when no other relevant TL is active
    return std::any_of(activeTl.begin(), activeTl.end(), [timeStep](const std::shared_ptr<TrafficLight> &tl) {
        return TurningDirections::all == tl->getDirection() and
               tl->getElementAtTime(timeStep).color != TrafficLightState::green;
    });
}