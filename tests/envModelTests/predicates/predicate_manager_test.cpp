//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "predicate_manager_test.h"
#include "../interfaces/utility_functions.h"
#include <commonroad_cpp/predicates/predicate_manager.h>
#include <filesystem>

TEST_F(PredicateManagerTest, ExtractRelevantPredicatesSingleThread) {
    int num_threads{1};
    extractRelevantPredicatesHelper(num_threads);
}

void PredicateManagerTest::extractRelevantPredicatesHelper(int num_threads) const {
    std::string benchmarkID{"DEU_test_safe_distance"};
    std::vector<std::string> dirs{TestUtils::getTestScenarioDirectory() + "/predicates"};
    EvaluationMode evalMode{EvaluationMode::directory};
    size_t egoVehicleID{0};
    bool performanceEvaluation{true};
    std::string outputDirectory{TestUtils::getTestScenarioDirectory() + "/"};
    std::string outputFileName{"test.txt"};
    PredicateManager predicateManager{
        PredicateManager(num_threads,
                         SimulationParameters(dirs, egoVehicleID, benchmarkID, evalMode, performanceEvaluation,
                                              outputDirectory, outputFileName),
                         {"keeps_safe_distance_prec", "in_standstill"})};
    predicateManager.extractPredicateSatisfaction();
    EXPECT_EQ(predicates["keeps_safe_distance_prec"]->getStatistics().numExecutions, 9588);
    EXPECT_EQ(predicates["keeps_safe_distance_prec"]->getStatistics().numSatisfaction, 8736);
    EXPECT_GT(predicates["keeps_safe_distance_prec"]->getStatistics().totalComputationTime, 0);
    EXPECT_GT(predicates["keeps_safe_distance_prec"]->getStatistics().minComputationTime, 0);
    EXPECT_GT(predicates["keeps_safe_distance_prec"]->getStatistics().maxComputationTime, 0);
    EXPECT_GE(predicates["keeps_safe_distance_prec"]->getStatistics().maxComputationTime,
              predicates["keeps_safe_distance_prec"]->getStatistics().minComputationTime);
    EXPECT_GE(predicates["keeps_safe_distance_prec"]->getStatistics().totalComputationTime,
              predicates["keeps_safe_distance_prec"]->getStatistics().maxComputationTime);
    EXPECT_EQ(predicates["in_standstill"]->getStatistics().numExecutions, 1173);
    EXPECT_EQ(predicates["in_standstill"]->getStatistics().numSatisfaction, 51);
    EXPECT_GT(predicates["in_standstill"]->getStatistics().totalComputationTime, 0);
    EXPECT_GT(predicates["in_standstill"]->getStatistics().minComputationTime, 0);
    EXPECT_GT(predicates["in_standstill"]->getStatistics().maxComputationTime, 0);
    EXPECT_GE(predicates["in_standstill"]->getStatistics().maxComputationTime,
              predicates["in_standstill"]->getStatistics().minComputationTime);
    EXPECT_GE(predicates["in_standstill"]->getStatistics().totalComputationTime,
              predicates["in_standstill"]->getStatistics().maxComputationTime);
    EXPECT_TRUE(std::filesystem::remove(TestUtils::getTestScenarioDirectory() + "/test.txt"));
    predicateManager.reset();
}

TEST_F(PredicateManagerTest, ExtractRelevantPredicatesMultiThread) {
    int num_threads{4};
    extractRelevantPredicatesHelper(num_threads);
}
