//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "predicate_manager_test.h"
#include "../interfaces/utility_functions.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"
#include <commonroad_cpp/predicates/predicate_manager.h>
#include <filesystem>

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

TEST_F(PredicateManagerTest, ExtractPredicateSatisfactionMultiThread) {
    int num_threads{4};
    extractRelevantPredicatesHelper(num_threads);
}

TEST_F(PredicateManagerTest, ExtractPredicateSatisfactionSingleThread) {
    int num_threads{1};
    extractRelevantPredicatesHelper(num_threads);
}

TEST_F(PredicateManagerTest, Reset) {
    int num_threads{4};
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
                         {"keeps_safe_distance_prec"})};
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
    predicateManager.reset();
    EXPECT_EQ(predicates["keeps_safe_distance_prec"]->getStatistics().numExecutions, 0);
    EXPECT_EQ(predicates["keeps_safe_distance_prec"]->getStatistics().numSatisfaction, 0);
    EXPECT_EQ(predicates["keeps_safe_distance_prec"]->getStatistics().totalComputationTime, 0);
    EXPECT_EQ(predicates["keeps_safe_distance_prec"]->getStatistics().minComputationTime, LONG_MAX);
    EXPECT_EQ(predicates["keeps_safe_distance_prec"]->getStatistics().maxComputationTime, LONG_MIN);
    EXPECT_TRUE(std::filesystem::remove(TestUtils::getTestScenarioDirectory() + "/test.txt"));
}

TEST_F(PredicateManagerTest, ReadConfigFileConstructor) {
    // Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string filePath;
    int argc{5};
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/../commonroad_cpp_tests/test_config.yaml"};
    std::vector<std::tuple<std::string, std::string>> replace{
        {"evaluation_mode: directory", "evaluation_mode: single_scenario"},
        {"directories: [ \"./tests/scenarios/predicates\" ]",
         "directories: [ " + TestUtils::getTestScenarioDirectory() + "/predicates/ ]"},
        {"output_directory: \"src/commonroad_cpp/predicates\"",
         "output_directory: " + TestUtils::getTestScenarioDirectory() + "/../commonroad_cpp_tests/ "}
    };
    TestUtils::copyAndReplaceContentInFile(TestUtils::getTestScenarioDirectory() +
                                               "/../../src/commonroad_cpp/default_config.yaml",
                                           pathToTestFile, replace);
    const char *array[5]{"myprogram", "--input-file", pathToTestFile.c_str(), "--t", "4"};
    char **argv{const_cast<char **>(array)};
    int error_code = InputUtils::readCommandLineValues(argc, argv, num_threads, filePath);
    EXPECT_EQ(error_code, 0);

    // Read and parse CommonRoad scenario file
    PredicateManager eval{num_threads, filePath};
    eval.extractPredicateSatisfaction();
    EXPECT_EQ(predicates["keeps_safe_distance_prec"]->getStatistics().numExecutions, 5610);
    EXPECT_EQ(predicates["keeps_safe_distance_prec"]->getStatistics().numSatisfaction, 4869);
    EXPECT_GT(predicates["keeps_safe_distance_prec"]->getStatistics().totalComputationTime, 0);
    EXPECT_GT(predicates["keeps_safe_distance_prec"]->getStatistics().minComputationTime, 0);
    EXPECT_GT(predicates["keeps_safe_distance_prec"]->getStatistics().maxComputationTime, 0);
    EXPECT_GE(predicates["keeps_safe_distance_prec"]->getStatistics().maxComputationTime,
              predicates["keeps_safe_distance_prec"]->getStatistics().minComputationTime);
    EXPECT_GE(predicates["keeps_safe_distance_prec"]->getStatistics().totalComputationTime,
              predicates["keeps_safe_distance_prec"]->getStatistics().maxComputationTime);
    EXPECT_EQ(predicates["in_intersection_main_area"]->getStatistics().numExecutions, 0);
    EXPECT_EQ(predicates["in_intersection_main_area"]->getStatistics().numSatisfaction, 0);
    EXPECT_EQ(predicates["in_intersection_main_area"]->getStatistics().totalComputationTime, 0);
    EXPECT_EQ(predicates["in_intersection_main_area"]->getStatistics().minComputationTime, LONG_MAX);
    EXPECT_EQ(predicates["in_intersection_main_area"]->getStatistics().maxComputationTime, LONG_MIN);
    EXPECT_TRUE(std::filesystem::remove(TestUtils::getTestScenarioDirectory() +
                                        "/../commonroad_cpp_tests/predicate_satisfaction.txt"));
    EXPECT_TRUE(
        std::filesystem::remove(TestUtils::getTestScenarioDirectory() + "/../commonroad_cpp_tests/test_config.yaml"));
}
