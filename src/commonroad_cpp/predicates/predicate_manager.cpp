//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "predicate_manager.h"
#include "../interfaces/standalone/command_line_input.h"
#include "../obstacle/obstacle.h"
#include "../world.h"
#include "commonroad_predicate.h"

void PredicateManager::extractPredicateSatisfaction() {
    for (const auto &sc : scenarios) {
        const auto &[obstacles, roadNetwork, timeStepSize] = CommandLine::getDataFromCommonRoad(sc);
        for (const auto &ego : obstacles) {
            std::vector<std::shared_ptr<Obstacle>> others;
            others.reserve(obstacles.size() - 1);
            for (const auto &obs : obstacles) {
                if (obs->getId() == ego->getId())
                    continue;
                others.push_back(obs);
            }
            // setObstacleProperties(ego, others);
            auto egoVehicles{std::vector<std::shared_ptr<Obstacle>>{ego}};
            auto world{std::make_shared<World>(0, roadNetwork, egoVehicles, others, timeStepSize)};
            for (size_t timeStep{ego->getCurrentState()->getTimeStep()}; timeStep <= ego->getLastTrajectoryTimeStep();
                 ++timeStep)
                for (const auto &[predName, pred] : predicates)
                    if (!pred->isVehicleDependent()) {
                        pred->statisticBooleanEvaluation(timeStep, world, ego, nullptr);
                    } else
                        for (const auto &obs : world->getObstacles()) {
                            pred->statisticBooleanEvaluation(timeStep, world, ego, obs);
                        }
        }
    }
}

PredicateManager::PredicateManager(int threads, const std::string &configPath)
    : numThreads(threads), simulationParameters(CommandLine::initialize(configPath)) {}
