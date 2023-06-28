//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_incoming.h"
#include "../../interfaces/utility_functions.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void IntersectionTestInitialization::setUpIncoming() {
    auto pathToTestFile = TestUtils::getTestScenarioDirectory() + "/DEU_incomingTest-1";
    const auto &[obstacles, roadnetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);
    intersection1 = roadnetwork->getIntersections()[0];
    intersection2 = roadnetwork->getIntersections()[1];
    incomingOne = roadnetwork->getIntersections()[0]->getIncomingGroups()[0];
    incomingTwo = roadnetwork->getIntersections()[0]->getIncomingGroups()[1];
    incomingThree = roadnetwork->getIntersections()[0]->getIncomingGroups()[2];
    roadNetwork = roadnetwork;
}

void IncomingTest::SetUp() { setUpIncoming(); }

TEST_F(IncomingTest, InitializationComplete) {
    EXPECT_EQ(incomingOne->getId(), 13);
    EXPECT_EQ(incomingTwo->getId(), 14);
    EXPECT_EQ(incomingThree->getId(), 15);

    EXPECT_EQ(incomingOne->getIncomingLanelets().size(), 1);
    EXPECT_EQ(incomingTwo->getIncomingLanelets().size(), 1);
    EXPECT_EQ(incomingThree->getIncomingLanelets().size(), 1);
    EXPECT_EQ(incomingOne->getIncomingLanelets().at(0)->getId(), 2);
    EXPECT_EQ(incomingTwo->getIncomingLanelets().at(0)->getId(), 3);
    EXPECT_EQ(incomingThree->getIncomingLanelets().at(0)->getId(), 6);

    EXPECT_EQ(incomingOne->getLeftOutgoings().size(), 0);
    EXPECT_EQ(incomingTwo->getLeftOutgoings().size(), 1); // TODO 0 and 2 straight
    EXPECT_EQ(incomingThree->getLeftOutgoings().size(), 1);
    EXPECT_EQ(incomingTwo->getLeftOutgoings().at(0)->getId(), 101);
    EXPECT_EQ(incomingThree->getLeftOutgoings().at(0)->getId(), 121);

    EXPECT_EQ(incomingOne->getRightOutgoings().size(), 1);
    EXPECT_EQ(incomingTwo->getRightOutgoings().size(), 0);
    EXPECT_EQ(incomingThree->getRightOutgoings().size(), 1);
    EXPECT_EQ(incomingOne->getRightOutgoings().at(0)->getId(), 111);
    EXPECT_EQ(incomingThree->getRightOutgoings().at(0)->getId(), 91);

    EXPECT_EQ(incomingOne->getStraightOutgoings().size(), 1);
    EXPECT_EQ(incomingTwo->getStraightOutgoings().size(), 1);
    EXPECT_EQ(incomingThree->getStraightOutgoings().size(), 0);
    EXPECT_EQ(incomingOne->getStraightOutgoings().at(0)->getId(), 81);
    EXPECT_EQ(incomingTwo->getStraightOutgoings().at(0)->getId(), 71);

    incomingOne->setOncomings({incomingTwo->getStraightOutgoings()});
    incomingTwo->setOncomings({incomingOne->getStraightOutgoings()});

    EXPECT_EQ(incomingOne->getOncomings().size(), 1);
    EXPECT_EQ(incomingTwo->getOncomings().size(), 1);
    EXPECT_EQ(incomingThree->getOncomings().size(), 0);
    EXPECT_EQ(incomingOne->getOncomings().at(0)->getId(), 71);
    EXPECT_EQ(incomingTwo->getOncomings().at(0)->getId(), 81);

    incomingOne->setOncomings({incomingTwo->getStraightOutgoings()});
    incomingTwo->setOncomings({incomingOne->getStraightOutgoings()});
}