//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_stop_line.h"
#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"

void StopLineTest::SetUp() {
    const auto *trafficSignIDLookupTable = TrafficSignLookupTableByCountry.at(SupportedTrafficSignCountry::GERMANY);
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
    size_t trafficSignId = 5000;
    auto trafficSignLightVertex{vertex{0, 0}};
    std::vector<std::string> trafficSignElementValues;
    auto trafficSignElements{std::vector<std::shared_ptr<TrafficSignElement>>{std::make_shared<TrafficSignElement>(
        trafficSignIDLookupTable->at(TrafficSignTypes::MAX_SPEED), trafficSignElementValues)}};
    trafficSign = std::make_shared<TrafficSign>(trafficSignId, trafficSignElements, trafficSignLightVertex, false);
    size_t trafficLightId = 6000;
    std::vector<TrafficLightCycleElement> trafficLightCycleElement{{TrafficLightState::green, 20}};
    trafficLight = std::make_shared<TrafficLight>(trafficLightId, trafficLightCycleElement, 0, TurningDirection::all,
                                                  true, trafficSignLightVertex);
    stopLine1 = StopLine(pointsStopLine1, {trafficSign}, {trafficLight}, lineMarkingStopLine1);
    stopLine2 = StopLine(pointsStopLine2, {trafficSign}, lineMarkingStopLine2);
    stopLine3 = StopLine(pointsStopLine3, {trafficLight}, lineMarkingStopLine3);
    stopLine4 = StopLine();
    stopLine5 = StopLine();
    stopLine4.setLineMarking(lineMarkingStopLine4);
    stopLine4.setPoints(pointsStopLine4);
    stopLine4.setTrafficLights({trafficLight});
    stopLine4.setTrafficSigns({trafficSign});
    stopLine5.setLineMarking(lineMarkingStopLine5);
    stopLine5.setPoints(pointsStopLine5);
    stopLine5.addTrafficLight(trafficLight);
    stopLine5.addTrafficSign(trafficSign);
}

TEST_F(StopLineTest, InitializationComplete) {
    EXPECT_EQ(stopLine1.getLineMarking(), lineMarkingStopLine1);
    EXPECT_EQ(stopLine1.getPoints().size(), pointsStopLine1.size());
    EXPECT_EQ(stopLine1.getPoints().at(0).x, pointsStopLine1.at(0).x);
    EXPECT_EQ(stopLine1.getPoints().at(0).y, pointsStopLine1.at(0).y);
    EXPECT_EQ(stopLine1.getPoints().at(1).x, pointsStopLine1.at(1).x);
    EXPECT_EQ(stopLine1.getPoints().at(1).y, pointsStopLine1.at(1).y);
    EXPECT_EQ(stopLine1.getTrafficSigns().size(), 1);
    EXPECT_EQ(stopLine1.getTrafficSigns().at(0)->getId(), 5000);
    EXPECT_EQ(stopLine1.getTrafficLights().size(), 1);
    EXPECT_EQ(stopLine1.getTrafficLights().at(0)->getId(), 6000);

    EXPECT_EQ(stopLine2.getLineMarking(), lineMarkingStopLine2);
    EXPECT_EQ(stopLine2.getPoints().size(), pointsStopLine2.size());
    EXPECT_EQ(stopLine2.getPoints().at(0).x, pointsStopLine2.at(0).x);
    EXPECT_EQ(stopLine2.getPoints().at(0).y, pointsStopLine2.at(0).y);
    EXPECT_EQ(stopLine2.getPoints().at(1).x, pointsStopLine2.at(1).x);
    EXPECT_EQ(stopLine2.getPoints().at(1).y, pointsStopLine2.at(1).y);
    EXPECT_EQ(stopLine2.getTrafficSigns().size(), 1);
    EXPECT_EQ(stopLine2.getTrafficSigns().at(0)->getId(), 5000);
    EXPECT_EQ(stopLine2.getTrafficLights().size(), 0);

    EXPECT_EQ(stopLine3.getLineMarking(), lineMarkingStopLine3);
    EXPECT_EQ(stopLine3.getPoints().size(), pointsStopLine3.size());
    EXPECT_EQ(stopLine3.getPoints().at(0).x, pointsStopLine3.at(0).x);
    EXPECT_EQ(stopLine3.getPoints().at(0).y, pointsStopLine3.at(0).y);
    EXPECT_EQ(stopLine3.getPoints().at(1).x, pointsStopLine3.at(1).x);
    EXPECT_EQ(stopLine3.getPoints().at(1).y, pointsStopLine3.at(1).y);
    EXPECT_EQ(stopLine3.getTrafficSigns().size(), 0);
    EXPECT_EQ(stopLine3.getTrafficLights().size(), 1);
    EXPECT_EQ(stopLine3.getTrafficLights().at(0)->getId(), 6000);

    EXPECT_EQ(stopLine4.getLineMarking(), lineMarkingStopLine4);
    EXPECT_EQ(stopLine4.getPoints().size(), pointsStopLine4.size());
    EXPECT_EQ(stopLine4.getPoints().at(0).x, pointsStopLine4.at(0).x);
    EXPECT_EQ(stopLine4.getPoints().at(0).y, pointsStopLine4.at(0).y);
    EXPECT_EQ(stopLine4.getPoints().at(1).x, pointsStopLine4.at(1).x);
    EXPECT_EQ(stopLine4.getPoints().at(1).y, pointsStopLine4.at(1).y);
    EXPECT_EQ(stopLine4.getTrafficSigns().size(), 1);
    EXPECT_EQ(stopLine4.getTrafficSigns().at(0)->getId(), 5000);
    EXPECT_EQ(stopLine4.getTrafficLights().size(), 1);
    EXPECT_EQ(stopLine4.getTrafficLights().at(0)->getId(), 6000);

    EXPECT_EQ(stopLine5.getLineMarking(), lineMarkingStopLine5);
    EXPECT_EQ(stopLine5.getPoints().size(), pointsStopLine5.size());
    EXPECT_EQ(stopLine5.getPoints().at(0).x, pointsStopLine5.at(0).x);
    EXPECT_EQ(stopLine5.getPoints().at(0).y, pointsStopLine5.at(0).y);
    EXPECT_EQ(stopLine5.getPoints().at(1).x, pointsStopLine5.at(1).x);
    EXPECT_EQ(stopLine5.getPoints().at(1).y, pointsStopLine5.at(1).y);
    EXPECT_EQ(stopLine5.getTrafficSigns().size(), 1);
    EXPECT_EQ(stopLine5.getTrafficSigns().at(0)->getId(), 5000);
    EXPECT_EQ(stopLine5.getTrafficLights().size(), 1);
    EXPECT_EQ(stopLine5.getTrafficLights().at(0)->getId(), 6000);
}