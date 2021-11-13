//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "predicate_manager.h"
#include <commonroad_cpp/interfaces/standalone/command_line_input.h>
#include "../obstacle/obstacle.h"
#include "../world.h"

void PredicateManager::extractPredicateSatisfaction() {
    for (const auto &sc : scenarios) {
        const auto &[obstacles, roadNetwork, timeStepSize] = CommandLine::getDataFromCommonRoad(sc);
         for (const auto &ego : obstacles){
             std::vector<std::shared_ptr<Obstacle>> others;
             others.reserve(obstacles.size() - 1);
             for (const auto &obs : obstacles) {
                 if (obs->getId() == ego->getId())
                     continue;
                 others.push_back(obs);
             }
             //setObstacleProperties(ego, others);
             auto egoVehicles{std::vector<std::shared_ptr<Obstacle>>{ego}};
             auto world{std::make_shared<World>(0, roadNetwork, egoVehicles, others, timeStepSize)};
         }
    }
}
