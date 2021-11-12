//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

#include "predicate_config.h"
#include <vector>

class PredicateManager {
  public:
    void extractPredicateSatisfaction();

  private:
    std::map<std::string, PredicateSatisfaction> predicateSatisfaction;
    std::vector<std::string> scenarios;
    int numThreads;
};
