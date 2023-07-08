#include "test_incoming.h"
#include "../../interfaces/utility_functions.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void IntersectionTestInitialization::setUpIncoming() {
    auto pathToTestFile = TestUtils::getTestScenarioDirectory() + "/DEU_IncomingTest-1/DEU_IncomingTest-1_1_T-1.pb";
    const auto &[obstacles, roadnetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);
    auto pathToCrosswalkFile = TestUtils::getTestScenarioDirectory() + "/DEU_Crosswalk-1/DEU_Crosswalk-1_1_T-1.pb";
        const auto &[obstacles2, roadnetwork2, timeStepSize2] = InputUtils::getDataFromCommonRoad(pathToCrosswalkFile);
    intersection1 = roadnetwork->getIntersections()[0];
    intersection2 = roadnetwork2->getIntersections()[0];
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

TEST_F(IncomingTest, LeftOf) {
    EXPECT_EQ(incomingOne->getIsLeftOf()->getId(), 15);
    EXPECT_EQ(incomingTwo->getIsLeftOf(), nullptr);
    EXPECT_EQ(incomingThree->getIsLeftOf()->getId(), 14);
}