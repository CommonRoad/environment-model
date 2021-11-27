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
#include <limits>
#include <random>
#include "yaml-cpp/yaml.h"

void PredicateManager::extractPredicateSatisfaction() {
    auto rng = std::default_random_engine {};
    // evaluate scenarios
    omp_set_num_threads(numThreads);
#pragma omp parallel for schedule(guided) shared(scenarios, predicates, rng) default(none)
    for (size_t i = 0; i < scenarios.size(); i++) {
        auto sc{scenarios.at(i)};
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
            omp_set_num_threads(numThreads);
#pragma omp parallel for schedule(guided) shared(predicates, world, sc, rng) firstprivate(ego) default(none)
            for (size_t timeStep = ego->getCurrentState()->getTimeStep(); timeStep <= ego->getLastTrajectoryTimeStep();
                 ++timeStep) {

                std::shuffle(std::begin(relevantPredicates), std::end(relevantPredicates), rng);
                for (const auto &predName : relevantPredicates) {
                    auto pred{predicates[predName]};
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
        }
    }
    writeFile();
}

void PredicateManager::writeFile() {
    std::ofstream file;
    double globalMinExecutionTime{std::numeric_limits<double>::max()};
    double globalMaxExecutionTime{std::numeric_limits<double>::lowest()};
    double globalMinAvgExecutionTime{std::numeric_limits<double>::max()};
    double globalMaxAvgExecutionTime{std::numeric_limits<double>::lowest()};
    file.open(simulationParameters.outputDirectory + "/" + simulationParameters.outputFileName);
    file << "Predicate Name - Num. Satisfactions - Num. Executions - Satisfaction in % - Max. Comp. Time - "
            "Min. Comp. Time - Avg. Comp. Time - Weight Max. Comp. Time - Weight Avg. Comp. Time \n";
    std::map<std::string, std::tuple<size_t, size_t, double, double, double, double>> predicateStatistics;
    for (const auto &predName : relevantPredicates) {
        auto pred{predicates[predName]};
        auto minExecutionTime{static_cast<double>(predicates[predName]->getStatistics().minComputationTime) / 1e6};
        auto maxExecutionTime{static_cast<double>(predicates[predName]->getStatistics().maxComputationTime) / 1e6};
        auto avgExecutionTime{(static_cast<double>(predicates[predName]->getStatistics().totalComputationTime) / 1e6) /
                              static_cast<double>(pred->getStatistics().numExecutions)};
        if (minExecutionTime < globalMinExecutionTime)
            globalMinExecutionTime = minExecutionTime;
        if (maxExecutionTime > globalMaxExecutionTime)
            globalMaxExecutionTime = maxExecutionTime;
        if (avgExecutionTime < globalMinAvgExecutionTime)
            globalMinAvgExecutionTime = avgExecutionTime;
        if (avgExecutionTime > globalMaxAvgExecutionTime)
            globalMaxAvgExecutionTime = avgExecutionTime;
        predicateStatistics.insert({predName,
                                    {pred->getStatistics().numSatisfaction, pred->getStatistics().numExecutions,
                                     static_cast<double>(pred->getStatistics().numSatisfaction) /
                                         static_cast<double>(pred->getStatistics().numExecutions),
                                     maxExecutionTime, minExecutionTime, avgExecutionTime}});
    }
    for (const auto &[predName, pred] : predicateStatistics) {
        file << predName << ": " << std::get<0>(pred) << " - " << std::get<1>(pred) << " - " << std::get<2>(pred)
             << " - " << std::get<3>(pred) << " [ms] - " << std::get<4>(pred) << " [ms] - " << std::get<5>(pred)
             << " [ms] - "
             << (std::get<3>(pred) - globalMinExecutionTime) / (globalMaxExecutionTime - globalMinExecutionTime)
             << " - "
             << (std::get<5>(pred) - globalMinAvgExecutionTime) /
                    (globalMaxAvgExecutionTime - globalMinAvgExecutionTime)
             << "\n";
    }
    file << "Global Max. Comp. Time: " << std::to_string(globalMaxExecutionTime) << '\n';
    file << "Global Min. Comp. Time: " << std::to_string(globalMinExecutionTime) << '\n';
    file << "Global Max. Avg Comp. Time: " << std::to_string(globalMaxAvgExecutionTime) << '\n';
    file << "Global Min. Avg Comp. Time: " << std::to_string(globalMinAvgExecutionTime) << '\n';
    file.close();
}

PredicateManager::PredicateManager(int threads, const std::string &configPath)
    : numThreads(threads), simulationParameters(CommandLine::initializeSimulationParameters(configPath)){
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
    extractRelevantPredicates(configPath);
}

void PredicateManager::extractRelevantPredicates(const std::string &configPath) {
    YAML::Node config = YAML::LoadFile(configPath);
    auto relevantPredicateSets{config["predicate_eval"]["relevant_predicate_sets"].as<std::vector<std::string>>()};
    for(const auto& predicateSet : relevantPredicateSets){
       auto relevantSetPredicates{config["predicate_eval"]["predicate_category"][predicateSet].as<std::vector<std::string>>()};
       for(const auto &pred : relevantSetPredicates)
           relevantPredicates.push_back(pred);
   }
}
