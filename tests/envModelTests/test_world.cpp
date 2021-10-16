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
    std::array<std::string, 6> scenarios{"ZAM_Urban-2_1.xml",      "USA_Peach-2_1_T-1.xml",
                                         "USA_Peach-4_1_T-1.xml",  "ESP_Almansa-2_2_T-1.xml",
                                         "ITA_Foggia-7_2_T-1.xml", "ARG_Carcarana-6_5_T-1.xml"};
    for (const auto &sc : scenarios) {
        std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + sc};
        const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
            CommandLine::getDataFromCommonRoad(pathToTestFileOne);
        EXPECT_NO_THROW(auto world{World(0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne)});
    }
}

// TEST_F(WorldTest, TestScenariosInValid) {
//    std::string scenario{"ARG_Carcarana-6_5_T-1.xml"};
//    size_t obstacleId{31};
//    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + scenario};
//    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
//    CommandLine::getDataFromCommonRoad(pathToTestFileOne); auto world =
//        World(0, roadNetworkScenarioOne, {obstacle_operations::getObstacleById(obstaclesScenarioOne, obstacleId)},
//        {}, timeStepSizeOne);
//    auto obs{world.findObstacle(obstacleId)};
//    for (const auto &t : obs->getTimeSteps())
//        EXPECT_THROW(auto ref{obs->getReferenceLane(t)}, std::runtime_error);
//}

TEST_F(WorldTest, TestSingleScenarioObstacle) {
    std::string scenario{"BEL_Zwevegem-1_5_T-1.xml"};
    size_t obstacleId{325};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + scenario};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        CommandLine::getDataFromCommonRoad(pathToTestFileOne);
    auto world = World(0, roadNetworkScenarioOne,
                       {obstacle_operations::getObstacleById(obstaclesScenarioOne, obstacleId)}, {}, timeStepSizeOne);
    auto obs{world.findObstacle(obstacleId)};
    for (const auto &t : obs->getTimeSteps())
        EXPECT_NO_THROW(auto ref{obs->getReferenceLane(t)});
}

TEST_F(WorldTest, TestSingleScenario) {
    std::string scenario{"USA_Peach-2_1_T-1.xml"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + scenario};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        CommandLine::getDataFromCommonRoad(pathToTestFileOne);
    EXPECT_NO_THROW(auto world{World(0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne)});
    for (const auto &obs : obstaclesScenarioOne)
        for (const auto &t : obs->getTimeSteps())
            EXPECT_NO_THROW(obs->getReferenceLane(t));
}

// TEST_F(WorldTest, TestAllScenarios) {
//    int numThreads{6};
//    // std::string path{"/media/sebastian/TUM/06_code/cps/scenarios"};
//    std::string path{"/media/sebastian/TUM/06_code/cps/scenarios"};
//    // std::array<std::string, 1> scenarios{"/cr-scenarios/scenarios/scenario-factory"};
//    std::array<std::string, 1> scenarios{"/Frankfurt"};
//    for (size_t i{0}; i < scenarios.size(); ++i)
//        scenarios[i] = path + scenarios[i];
//
//    for (auto const &dir : scenarios) {
//        std::vector<std::string> fileNames;
//        for (directory_iterator itr(dir); itr != directory_iterator(); ++itr)
//            if (boost::algorithm::ends_with(itr->path().string(), ".xml"))
//                fileNames.push_back(itr->path().string());
//        //        omp_set_num_threads(numThreads);
//        //#pragma omp parallel for schedule(guided) shared(fileNames) default(none)
//        for (auto &fileName : fileNames) {
//            const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
//            CommandLine::getDataFromCommonRoad(fileName); auto world{World(0, roadNetworkScenarioOne,
//            obstaclesScenarioOne, {}, timeStepSizeOne)}; for (const auto &obs : obstaclesScenarioOne)
//                for (const auto &t : obs->getTimeSteps())
//                    try {
//                        obs->getReferenceLane(t);
//                    } catch (const std::runtime_error &re) {
//                        std::cerr << "Runtime error: " << re.what() << std::endl;
//                        std::cerr << "Scenario: " << fileName << std::endl;
//                    }
//        }
//    }
//}
