
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
    std::pair<vertex, vertex> pointsStopLine1;
    std::pair<vertex, vertex> pointsStopLine2;
    std::pair<vertex, vertex> pointsStopLine3;
    std::pair<vertex, vertex> pointsStopLine4;
    std::pair<vertex, vertex> pointsStopLine5;
    LineMarking lineMarkingStopLine1;
    LineMarking lineMarkingStopLine2;
    LineMarking lineMarkingStopLine3;
    LineMarking lineMarkingStopLine4;
    LineMarking lineMarkingStopLine5;

  private:
    void SetUp() override;
};
