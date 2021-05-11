//
// Created by sebastian on 07.12.20.
//

#include "test_lanelet_operations.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"

void LaneletOperationsTest::SetUp() {
    setUpLane();
    setUpRoadNetwork();
}

TEST_F(LaneletOperationsTest, MatchStringToLaneletType) {
    EXPECT_EQ(matchStringToLaneletType("interstate"), LaneletType::interstate);
    EXPECT_EQ(matchStringToLaneletType("urban"), LaneletType::urban);
    EXPECT_EQ(matchStringToLaneletType("crosswalk"), LaneletType::crosswalk);
    EXPECT_EQ(matchStringToLaneletType("busStop"), LaneletType::busStop);
    EXPECT_EQ(matchStringToLaneletType("country"), LaneletType::country);
    EXPECT_EQ(matchStringToLaneletType("highway"), LaneletType::highway);
    EXPECT_EQ(matchStringToLaneletType("driveWay"), LaneletType::driveWay);
    EXPECT_EQ(matchStringToLaneletType("mainCarriageWay"), LaneletType::mainCarriageWay);
    EXPECT_EQ(matchStringToLaneletType("accessRamp"), LaneletType::accessRamp);
    EXPECT_EQ(matchStringToLaneletType("exitRamp"), LaneletType::exitRamp);
    EXPECT_EQ(matchStringToLaneletType("shoulder"), LaneletType::shoulder);
    EXPECT_EQ(matchStringToLaneletType("bikeLane"), LaneletType::bikeLane);
    EXPECT_EQ(matchStringToLaneletType("sidewalk"), LaneletType::sidewalk);
    EXPECT_EQ(matchStringToLaneletType("busLane"), LaneletType::busLane);
    EXPECT_EQ(matchStringToLaneletType("test"), LaneletType::unknown);
}

TEST_F(LaneletOperationsTest, MatchStringToLineMarking) {
    EXPECT_EQ(matchStringToLineMarking("solid"), LineMarking::solid);
    EXPECT_EQ(matchStringToLineMarking("dashed"), LineMarking::dashed);
    EXPECT_EQ(matchStringToLineMarking("broad_solid"), LineMarking::broad_solid);
    EXPECT_EQ(matchStringToLineMarking("broad_dashed"), LineMarking::broad_dashed);
    EXPECT_EQ(matchStringToLineMarking("no_marking"), LineMarking::no_marking);
    EXPECT_EQ(matchStringToLineMarking("test"), LineMarking::unknown);
}

TEST_F(LaneletOperationsTest, CombineLaneletAndSuccessorsWithSameTypeToLane) {
    EXPECT_EQ(
        combineLaneletAndSuccessorsWithSameTypeToLane(laneletThree, LaneletType::mainCarriageWay)->getLanelet().getId(),
        roadNetwork->getLanes().at(0)->getLanelet().getId());
    compareVerticesVector(combineLaneletAndSuccessorsWithSameTypeToLane(laneletThree, LaneletType::mainCarriageWay)
                              ->getLanelet()
                              .getCenterVertices(),
                          roadNetwork->getLanes().at(0)->getLanelet().getCenterVertices());
    EXPECT_EQ(
        combineLaneletAndSuccessorsWithSameTypeToLane(laneletSeven, LaneletType::mainCarriageWay)->getLanelet().getId(),
        10);
    EXPECT_EQ(combineLaneletAndSuccessorsWithSameTypeToLane(laneletThree, LaneletType::urban)->getLanelet().getId(), 3);
}
