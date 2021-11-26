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
#include <fstream>
#include <iostream>

void PredicateManager::extractPredicateSatisfaction() {
    // evaluate scenarios
    for (const auto &sc : scenarios) {
        const auto &[obstacles, roadNetwork, timeStepSize] = CommandLine::getDataFromCommonRoad(sc);
        // evaluate all obstacles
        for (const auto &ego : obstacles) {
            if (ego->getIsStatic())
                continue;
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
                    try {
                        if (!pred->isVehicleDependent())
                            pred->statisticBooleanEvaluation(timeStep, world, ego, nullptr);
                        else
                            for (const auto &obs : world->getObstacles())
                                pred->statisticBooleanEvaluation(timeStep, world, ego, obs);
                    } catch (...) {
                        throw std::runtime_error("PredicateManager::extractPredicateSatisfaction - Scenario: " + sc +
                                                 " - ego vehicle: " + std::to_string(ego->getId()) +
                                                 " - time step:" + std::to_string(timeStep));
                    }
        }
    }
    writeFile();
}

void PredicateManager::writeFile() {
    std::ofstream file;
    file.open(simulationParameters.outputDirectory + "/" + simulationParameters.outputFileName);
    file << "Predicate Name - Number Satisfactions - Number Executions - Percentage Satisfaction - Max. Comp. Time - Min. Comp. Time - Avg. Comp. Time\n";
    for (const auto &[predName, pred] : predicates) {
        file << predName << ": " << pred->getStatistics().numSatisfaction << " - "
             << pred->getStatistics().numExecutions << " - "
             << static_cast<double>(pred->getStatistics().numSatisfaction) /
                    static_cast<double>(pred->getStatistics().numExecutions)
             << " - " << static_cast<double>(predicates[predName]->getStatistics().maxComputationTime) / 1e6
             << " [ms] - " << static_cast<double>(predicates[predName]->getStatistics().minComputationTime) / 1e6
             << " [ms] - "
             << (static_cast<double>(predicates[predName]->getStatistics().totalComputationTime) / 1e6) /
                    static_cast<double>(pred->getStatistics().numExecutions)
             << " [ms]\n";
    }
    file.close();
}

PredicateManager::PredicateManager(int threads, const std::string &configPath)
    : numThreads(threads), simulationParameters(CommandLine::initialize(configPath)) {
    EvaluationMode mode{simulationParameters.evaluationMode};
    std::string singleScenario{simulationParameters.benchmarkId};
    for (auto const &dir : simulationParameters.directoryPaths) {
        std::vector<std::string> fileNames{CommandLine::findRelevantScenarioFileNames(dir)};
        std::copy_if(fileNames.begin(), fileNames.end(), std::back_inserter(scenarios),
                     [mode, singleScenario](const std::string &name) {
                         return (mode == EvaluationMode::directory) or (mode == EvaluationMode::singleScenario and
                                                                        name.find(singleScenario) != std::string::npos);
                     });
    }
    switch (simulationParameters.evaluationMode) {
    case EvaluationMode::directory:

        break;
    case EvaluationMode::singleScenario:
        break;
    case EvaluationMode::singleVehicle:
        break;
    case EvaluationMode::directory_single_vehicle:
        break;
    }
}
