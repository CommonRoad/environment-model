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

TEST_F(WorldTest, TestScenarios) {
    std::array<std::string, 3> scenarios{"ZAM_Urban-2_1.xml", "USA_Peach-2_1_T-1.xml", "USA_Peach-4_1_T-1.xml"};
    for (const auto &sc : scenarios) {
        std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + sc};
        const auto &[obstaclesScenarioOne, roadNetworkScenarioOne] =
            CommandLine::getDataFromCommonRoad(pathToTestFileOne);
        EXPECT_NO_THROW(auto world{World(0, roadNetworkScenarioOne, obstaclesScenarioOne, {})});
    }
}

TEST_F(WorldTest, TestSingleScenarioObstacle) {
    std::string scenario{"DEU_Guetersloh-25_4_T-1.xml"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + scenario};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne] = CommandLine::getDataFromCommonRoad(pathToTestFileOne);
    EXPECT_NO_THROW(auto world{
        World(0, roadNetworkScenarioOne, {obstacle_operations::getObstacleById(obstaclesScenarioOne, 325)}, {})});
}

TEST_F(WorldTest, TestSingleScenario) {
    std::string scenario{"USA_Peach-4_1_T-1.xml"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + scenario};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne] = CommandLine::getDataFromCommonRoad(pathToTestFileOne);
    EXPECT_NO_THROW(auto world{World(0, roadNetworkScenarioOne, obstaclesScenarioOne, {})});
}