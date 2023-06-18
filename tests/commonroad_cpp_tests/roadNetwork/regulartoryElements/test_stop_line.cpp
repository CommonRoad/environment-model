//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_stop_line.h"
#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"

void StopLineTest::SetUp() {
    pointsStopLine1 = {{0.0, 0.0}, {0.0, 1.0}};
    pointsStopLine2 = {{1.0, 0.0}, {1.0, 1.0}};
    pointsStopLine3 = {{2.0, 0.0}, {2.0, 1.0}};
    pointsStopLine4 = {{3.0, 0.0}, {3.0, 1.0}};
    pointsStopLine5 = {{4.0, 0.0}, {4.0, 1.0}};
    lineMarkingStopLine1 = LineMarking::broad_dashed;
    lineMarkingStopLine2 = LineMarking::solid;
    lineMarkingStopLine3 = LineMarking::unknown;
    lineMarkingStopLine3 = LineMarking::dashed;
    lineMarkingStopLine3 = LineMarking::broad_dashed;
    stopLine1 = StopLine(pointsStopLine1, lineMarkingStopLine1);
    stopLine2 = StopLine(pointsStopLine2, lineMarkingStopLine2);
    stopLine3 = StopLine(pointsStopLine3, lineMarkingStopLine3);
    stopLine4 = StopLine();
    stopLine5 = StopLine();
    stopLine4.setLineMarking(lineMarkingStopLine4);
    stopLine4.setPoints(pointsStopLine4);
    stopLine5.setLineMarking(lineMarkingStopLine5);
    stopLine5.setPoints(pointsStopLine5);
}

TEST_F(StopLineTest, InitializationComplete) {
    EXPECT_EQ(stopLine1.getLineMarking(), lineMarkingStopLine1);
    EXPECT_EQ(stopLine1.getPoints().first.x, pointsStopLine1.first.x);
    EXPECT_EQ(stopLine1.getPoints().first.y, pointsStopLine1.first.y);
    EXPECT_EQ(stopLine1.getPoints().second.x, pointsStopLine1.second.x);
    EXPECT_EQ(stopLine1.getPoints().second.y, pointsStopLine1.second.y);

    EXPECT_EQ(stopLine2.getLineMarking(), lineMarkingStopLine2);
    EXPECT_EQ(stopLine2.getPoints().first.x, pointsStopLine2.first.x);
    EXPECT_EQ(stopLine2.getPoints().first.y, pointsStopLine2.first.y);
    EXPECT_EQ(stopLine2.getPoints().second.x, pointsStopLine2.second.x);
    EXPECT_EQ(stopLine2.getPoints().second.y, pointsStopLine2.second.y);

    EXPECT_EQ(stopLine3.getLineMarking(), lineMarkingStopLine3);
    EXPECT_EQ(stopLine3.getPoints().first.x, pointsStopLine3.first.x);
    EXPECT_EQ(stopLine3.getPoints().first.y, pointsStopLine3.first.y);
    EXPECT_EQ(stopLine3.getPoints().second.x, pointsStopLine3.second.x);
    EXPECT_EQ(stopLine3.getPoints().second.y, pointsStopLine3.second.y);

    EXPECT_EQ(stopLine4.getLineMarking(), lineMarkingStopLine4);
    EXPECT_EQ(stopLine4.getPoints().first.x, pointsStopLine4.first.x);
    EXPECT_EQ(stopLine4.getPoints().first.y, pointsStopLine4.first.y);
    EXPECT_EQ(stopLine4.getPoints().second.x, pointsStopLine4.second.x);
    EXPECT_EQ(stopLine4.getPoints().second.y, pointsStopLine4.second.y);

    EXPECT_EQ(stopLine5.getLineMarking(), lineMarkingStopLine5);
    EXPECT_EQ(stopLine5.getPoints().first.x, pointsStopLine5.first.x);
    EXPECT_EQ(stopLine5.getPoints().first.y, pointsStopLine5.first.y);
    EXPECT_EQ(stopLine5.getPoints().second.x, pointsStopLine5.second.x);
    EXPECT_EQ(stopLine5.getPoints().second.y, pointsStopLine5.second.y);
}