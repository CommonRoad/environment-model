#pragma once

#include <gtest/gtest.h>

#include "../roadNetwork/test_road_network.h"
#include "commonroad_cpp/obstacle/state.h"

class StateTestInitialization : public RoadNetworkTestInitialization {
  protected:
    int timeStepStateOne;
    double xPositionStateOne;
    double yPositionStateOne;
    double velocityStateOne;
    double accelerationStateOne;
    double lonPositionStateOne;
    double latPositionStateOne;
    double globalOrientationStateOne;
    double curvilinearOrientationStateOne;
    ValidStates validityStateOne;

    int timeStepStateTwo;
    double xPositionStateTwo;
    double yPositionStateTwo;
    double velocityStateTwo;
    double accelerationStateTwo;
    double lonPositionStateTwo;
    double latPositionStateTwo;
    double globalOrientationStateTwo;
    double curvilinearOrientationStateTwo;
    ValidStates validityStateTwo;

    int timeStepStateThree;
    double xPositionStateThree;
    double yPositionStateThree;
    double velocityStateThree;
    double accelerationStateThree;
    double lonPositionStateThree;
    double latPositionStateThree;
    double globalOrientationStateThree;
    double curvilinearOrientationStateThree;
    ValidStates validityStateThree;

    int timeStepStateFour;
    double xPositionStateFour;
    double yPositionStateFour;
    double velocityStateFour;
    double accelerationStateFour;
    double lonPositionStateFour;
    double latPositionStateFour;
    double globalOrientationStateFour;
    double curvilinearOrientationStateFour;
    ValidStates validityStateFour;

    int timeStepStateFive;
    double xPositionStateFive;
    double yPositionStateFive;
    double velocityStateFive;
    double accelerationStateFive;
    double lonPositionStateFive;
    double latPositionStateFive;
    double globalOrientationStateFive;
    double curvilinearOrientationStateFive;

    std::shared_ptr<State> stateOne;
    std::shared_ptr<State> stateTwo;
    std::shared_ptr<State> stateThree;
    std::shared_ptr<State> stateFour;
    std::shared_ptr<State> stateFive;

    void setUpStates();
};

class StateTest : public StateTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
