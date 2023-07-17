//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <algorithm>

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>
#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"

bool intersection_operations::onIncoming(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                         const std::shared_ptr<RoadNetwork> &roadNetwork) {
    auto lanelets{obs->getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    for (const auto &let : lanelets)
        if (std::any_of(let->getLaneletTypes().begin(), let->getLaneletTypes().end(),
                        [](const LaneletType typ) { return typ == LaneletType::incoming; }))
            return true;

    return false;
}

bool intersection_operations::checkSameIncoming(const std::shared_ptr<Lanelet> &letK,
                                                const std::shared_ptr<Lanelet> &letP,
                                                const std::shared_ptr<RoadNetwork> &roadNetwork) {
    auto simLaneletsK{lane_operations::combineLaneLanelets(lane_operations::combineLaneletAndPredecessorsToLane(letK))};
    auto simLaneletsP{lane_operations::combineLaneLanelets(lane_operations::combineLaneletAndPredecessorsToLane(letP))};
    for (const auto &laK : simLaneletsK) {
        if (!laK->hasLaneletType(LaneletType::incoming))
            continue;
        for (const auto &adjLet : lanelet_operations::adjacentLanelets(laK)) {
            if (std::any_of(simLaneletsP.begin(), simLaneletsP.end(), [adjLet](const std::shared_ptr<Lanelet> &exLet) {
                    return exLet->getId() == adjLet->getId();
                }))
                return true;
        }
    }
    return false;
}

void intersection_operations::findLeftOf(const std::shared_ptr<IncomingGroup> &origin, const std::shared_ptr<RoadNetwork> &roadNetwork) {
    if (!origin->getRightOutgoings().empty()) {
        auto out = roadNetwork->findOutgoingGroupByLanelet(origin->getRightOutgoings()[0]);
        if (out) // TODO solve problem for outgoings without incomings like ru10 oneWayStreetIn
            origin->setIsLeftOf(roadNetwork->findIncomingGroupByOutgoingGroup(out));
    }
}
