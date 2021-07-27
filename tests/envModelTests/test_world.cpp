//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_world.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/world.h"
#include "interfaces/utility_functions.h"
#include <array>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <omp.h>

using namespace boost::filesystem;

TEST_F(WorldTest, TestScenariosValid) {
    std::array<std::string, 1> scenarios{"USA_Peach-4_1_T-1.xml"};
    for (const auto &sc : scenarios) {
        std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + sc};
        const auto &[obstaclesScenarioOne, roadNetworkScenarioOne] =
            CommandLine::getDataFromCommonRoad(pathToTestFileOne);
        EXPECT_NO_THROW(auto world{World(0, roadNetworkScenarioOne, obstaclesScenarioOne, {})});
    }
}

TEST_F(WorldTest, TestScenariosInValid) {
    std::array<std::string, 1> scenarios{"ARG_Carcarana-6_5_T-1.xml"};
    for (const auto &sc : scenarios) {
        std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + sc};
        const auto &[obstaclesScenarioOne, roadNetworkScenarioOne] =
            CommandLine::getDataFromCommonRoad(pathToTestFileOne);
        EXPECT_THROW(auto world{World(0, roadNetworkScenarioOne, obstaclesScenarioOne, {})}, std::runtime_error);
    }
}

TEST_F(WorldTest, TestSingleScenarioObstacle) {
    std::string scenario{"USA_Peach-4_1_T-1.xml"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + scenario};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne] = CommandLine::getDataFromCommonRoad(pathToTestFileOne);
    EXPECT_NO_THROW(auto world{
        World(0, roadNetworkScenarioOne, {obstacle_operations::getObstacleById(obstaclesScenarioOne, 271)}, {})});
}

TEST_F(WorldTest, TestSingleScenario) {
    std::string scenario{"USA_Peach-2_1_T-1.xml"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + scenario};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne] = CommandLine::getDataFromCommonRoad(pathToTestFileOne);
    EXPECT_NO_THROW(auto world{World(0, roadNetworkScenarioOne, obstaclesScenarioOne, {})});
}

// TEST_F(WorldTest, TestAllScenarios) {
//    int numThreads{6};
//    std::string path{"/media/sebastian/TUM/06_code/cps/scenarios/cr-scenarios/scenarios"};
//    std::array<std::string, 1> scenarios{"/scenario-factory"};
//    for (size_t i{0}; i < scenarios.size(); ++i)
//        scenarios[i] = path + scenarios[i];
//
//    for (auto const &dir : scenarios) {
//        std::vector<std::string> fileNames;
//        for (directory_iterator itr(dir); itr != directory_iterator(); ++itr)
//            if (boost::algorithm::ends_with(itr->path().string(), ".xml"))
//                fileNames.push_back(itr->path().string());
////        omp_set_num_threads(numThreads);
////#pragma omp parallel for schedule(guided) shared(fileNames, results) firstprivate(monitor) default(none)
//        for (size_t i = 0; i < fileNames.size(); i++) {
//            const auto &[obstaclesScenarioOne, roadNetworkScenarioOne] =
//            CommandLine::getDataFromCommonRoad(fileNames.at(i)); EXPECT_NO_THROW(auto world{World(0,
//            roadNetworkScenarioOne, obstaclesScenarioOne, {})});
//        }
//    }
//}