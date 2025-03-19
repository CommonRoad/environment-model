#pragma once

#include <gtest/gtest.h>

#include "commonroad_cpp/obstacle/signal_state.h"

class SignalStateTestInitialization {
  protected:
    std::shared_ptr<SignalState> signalStateOne;
    std::shared_ptr<SignalState> signalStateTwo;

    size_t timeStepSignalStateOne;
    bool hornSignalStateOne;
    bool indicatorLeftSignalStateOne;
    bool indicatorRightSignalStateOne;
    bool brakingLightsSignalStateOne;
    bool hazardWarningLightsSignalStateOne;
    bool flashingBlueLightsSignalStateOne;

    size_t timeStepSignalStateTwo;
    bool hornSignalStateTwo;
    bool indicatorLeftSignalStateTwo;
    bool indicatorRightSignalStateTwo;
    bool brakingLightsSignalStateTwo;
    bool hazardWarningLightsSignalStateTwo;
    bool flashingBlueLightsSignalStateTwo;

    void setUpSignalStates();
};

class SignalStateTest : public SignalStateTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
