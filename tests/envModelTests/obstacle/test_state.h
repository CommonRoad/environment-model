//
// Created by sebastian on 20.12.20.
//

#ifndef ENV_MODEL_TEST_STATE_H
#define ENV_MODEL_TEST_STATE_H

#pragma once
#include <gtest/gtest.h>

#include "obstacle/state.h"

class StateTestInitialization {
protected:
    int timeStepStateOne;
    double xPositionStateOne;
    double yPositionStateOne;
    double velocityStateOne;
    double accelerationStateOne;
    double lonPositionStateOne;
    double latPositionStateOne;
    double orientationStateOne;
    ValidStates validityStateOne;

    int timeStepStateTwo;
    double xPositionStateTwo;
    double yPositionStateTwo;
    double velocityStateTwo;
    double accelerationStateTwo;
    double lonPositionStateTwo;
    double latPositionStateTwo;
    double orientationStateTwo;
    ValidStates validityStateTwo;

    int timeStepStateThree;
    double xPositionStateThree;
    double yPositionStateThree;
    double velocityStateThree;
    double accelerationStateThree;
    double lonPositionStateThree;
    double latPositionStateThree;
    double orientationStateThree;
    ValidStates validityStateThree;

    int timeStepStateFour;
    double xPositionStateFour;
    double yPositionStateFour;
    double velocityStateFour;
    double accelerationStateFour;
    double lonPositionStateFour;
    double latPositionStateFour;
    double orientationStateFour;
    ValidStates validityStateFour;

    int timeStepStateFive;
    double xPositionStateFive;
    double yPositionStateFive;
    double velocityStateFive;
    double accelerationStateFive;
    double lonPositionStateFive;
    double latPositionStateFive;
    double orientationStateFive;
    ValidStates validityStateFive;

    int timeStepStateSix;
    double xPositionStateSix;
    double yPositionStateSix;
    double velocityStateSix;
    double accelerationStateSix;
    double lonPositionStateSix;
    double latPositionStateSix;
    double orientationStateSix;
    ValidStates validityStateSix;

    State stateOne;
    State stateTwo;
    State stateThree;
    State stateFour;
    State stateFive;
    State stateSix;

    void setUpStates();

};

class StateTest : public StateTestInitialization, public testing::Test {
private:
    void SetUp() override;
};

#endif //ENV_MODEL_TEST_STATE_H
