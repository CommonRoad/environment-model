//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "predicate_manager.h"
#include <commonroad_cpp/interfaces/standalone/command_line_input.h>

void PredicateManager::extractPredicateSatisfaction() {
    for (const auto &sc : scenarios) {
        const auto &[obstacles, roadNetwork, timeStepSize] = CommandLine::getDataFromCommonRoad(sc);

        // for
        //   auto world{std::make_shared<World>(0, roadNetwork, egoVehicles, others, timeStepSize)};
    }
}
