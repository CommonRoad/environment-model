//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

#include "commonroad_predicate.h"
#include "predicate_config.h"
#include <memory>
#include <vector>

class PredicateManager {
  public:
    /**
     * Constructor
     *
     * @param threads Number of threads which can be used.
     * @param configPath Path to configuration file which contains information about operation mode and traffic rules.
     */
    PredicateManager(int threads, const std::string &configPath);

    PredicateManager(int threads, SimulationParameters simulationParameters,
                     std::vector<std::string> relevantPredicates);

    void extractPredicateSatisfaction();

    void reset();

  private:
    std::map<std::string, PredicateSatisfaction> predicateSatisfaction;
    std::vector<std::string> scenarios;
    int numThreads;
    SimulationParameters simulationParameters;
    std::vector<std::string> relevantPredicates;

    void writeFile();
    void extractRelevantPredicates(const std::string &configPath);
    void extractScenarios();
};
