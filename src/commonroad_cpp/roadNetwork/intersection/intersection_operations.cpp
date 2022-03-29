//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <algorithm>

#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>

#include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"
#include "intersection_operations.h"

bool intersection_operations::onIncoming(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                         const std::shared_ptr<RoadNetwork> &roadNetwork) {
    auto lanelets{obs->getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    for (const auto &let : lanelets)
        if (std::any_of(let->getLaneletTypes().begin(), let->getLaneletTypes().end(),
                        [](const LaneletType typ) { return typ == LaneletType::incoming; }))
            return true;

    return false;
}