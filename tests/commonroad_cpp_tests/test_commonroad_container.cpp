//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_commonroad_container.h"
#include "commonroad_cpp/commonroad_container.h"
#include "interfaces/utility_functions.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void CommonRoadContainerTest::SetUp() {
    std::array<std::string, 6> scenarios{"ZAM_Urban-2_1.xml",      "USA_Peach-4_1_T-1.xml",
                                         "USA_Peach-2_1_T-1.xml",  "ESP_Almansa-2_2_T-1.xml",
                                         "ITA_Foggia-7_2_T-1.xml", "ARG_Carcarana-6_5_T-1.xml"};
    size_t scenarioId{1};
    std::shared_ptr<CommonRoadContainer> eval = CommonRoadContainer::getInstance();
    for (const auto &scen : scenarios) {
        std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + scen};
        const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFileOne);
        eval->registerScenario(scenarioId, 0, timeStepSize, roadNetwork, obstacles, {});
        scenarioId++;
    }
}

TEST_F(CommonRoadContainerTest, RegisterScenario) {
    std::shared_ptr<CommonRoadContainer> eval = CommonRoadContainer::getInstance();
    EXPECT_THROW(eval->findWorld(10), std::logic_error);
    EXPECT_NO_THROW(eval->findWorld(1));
    EXPECT_NO_THROW(eval->findWorld(2));
    EXPECT_NO_THROW(eval->findWorld(3));
    EXPECT_NO_THROW(eval->findWorld(4));
    EXPECT_NO_THROW(eval->findWorld(5));
    EXPECT_NO_THROW(eval->findWorld(6));
}

TEST_F(CommonRoadContainerTest, RemoveScenario) {
    std::shared_ptr<CommonRoadContainer> eval = CommonRoadContainer::getInstance();
    EXPECT_THROW(eval->removeScenario(10), std::logic_error);
    EXPECT_NO_THROW(eval->findWorld(6));
    EXPECT_NO_THROW(eval->removeScenario(1));
    EXPECT_THROW(eval->findWorld(1), std::logic_error);
    EXPECT_NO_THROW(eval->removeScenario(2));
    EXPECT_THROW(eval->findWorld(2), std::logic_error);
    EXPECT_NO_THROW(eval->removeScenario(3));
    EXPECT_THROW(eval->findWorld(3), std::logic_error);
    EXPECT_NO_THROW(eval->removeScenario(4));
    EXPECT_THROW(eval->findWorld(4), std::logic_error);
    EXPECT_NO_THROW(eval->removeScenario(5));
    EXPECT_THROW(eval->findWorld(5), std::logic_error);
    EXPECT_NO_THROW(eval->removeScenario(6));
    EXPECT_THROW(eval->findWorld(6), std::logic_error);
}
