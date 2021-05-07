//
// Created by sebastian on 20.12.20.
//

#ifndef ENV_MODEL_TEST_STATE_H
#define ENV_MODEL_TEST_STATE_H

#include <gtest/gtest.h>

#include "../roadNetwork/test_road_network.h"
#include "obstacle/state.h"

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
    ValidStates validityStateFive;

    int timeStepStateSix;
    double xPositionStateSix;
    double yPositionStateSix;
    double velocityStateSix;
    double accelerationStateSix;
    double lonPositionStateSix;
    double latPositionStateSix;
    double globalOrientationStateSix;
    double curvilinearOrientationStateSix;
    ValidStates validityStateSix;

    std::shared_ptr<State> stateOne;
    std::shared_ptr<State> stateTwo;
    std::shared_ptr<State> stateThree;
    std::shared_ptr<State> stateFour;
    std::shared_ptr<State> stateFive;
    std::shared_ptr<State> stateSix;

    void setUpStates();
};

class StateTest : public StateTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};

#endif // ENV_MODEL_TEST_STATE_H
