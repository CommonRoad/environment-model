//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

#include "commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h"
#include <gtest/gtest.h>

class StopLineTest : public testing::Test {
  protected:
    StopLine stopLine1;
    StopLine stopLine2;
    StopLine stopLine3;
    StopLine stopLine4;
    StopLine stopLine5;
    std::shared_ptr<TrafficSign> trafficSign;
    std::shared_ptr<TrafficLight> trafficLight;
    std::vector<vertex> pointsStopLine1;
    std::vector<vertex> pointsStopLine2;
    std::vector<vertex> pointsStopLine3;
    std::vector<vertex> pointsStopLine4;
    std::vector<vertex> pointsStopLine5;
    LineMarking lineMarkingStopLine1;
    LineMarking lineMarkingStopLine2;
    LineMarking lineMarkingStopLine3;
    LineMarking lineMarkingStopLine4;
    LineMarking lineMarkingStopLine5;

  private:
    void SetUp() override;
};
