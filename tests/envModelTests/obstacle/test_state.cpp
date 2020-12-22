//
// Created by sebastian on 20.12.20.
//

#include "test_state.h"

void StateTestInitialization::setUpStates(){
    xPositionStateOne = 2.5;
    yPositionStateOne = 0.5;
    velocityStateOne = 2;
    accelerationStateOne = 1.0;
    lonPositionStateOne = 2.5;
    latPositionStateOne = 0.5;
    orientationStateOne = 0.01;
    timeStepStateOne = 0;
    validityStateOne = ValidStates{true, true, true, true,
                                   true, true, true};

    xPositionStateTwo = 3.5;
    yPositionStateTwo = 2;
    velocityStateTwo = 20.0;
    accelerationStateTwo = 0.0;
    lonPositionStateTwo = 3.5;
    latPositionStateTwo = 2.0;
    orientationStateTwo = -0.01;
    timeStepStateTwo = 1;
    validityStateTwo = ValidStates{true, true, true, true,
                                   true, true, true};

    xPositionStateThree = 4.0;
    yPositionStateThree = -0.5;
    velocityStateThree = 0.0;
    accelerationStateThree = -0.6;
    lonPositionStateThree = 3.5;
    latPositionStateThree = -0.5;
    orientationStateThree = 0.0;
    timeStepStateThree = 2;
    validityStateThree = ValidStates{true, true, true, true,
                                   true, true, true};

    xPositionStateFour = 5.0;
    yPositionStateFour = -0.0;
    velocityStateFour = 50.0;
    accelerationStateFour = 0.0;
    lonPositionStateFour = 5;
    latPositionStateFour = 0;
    orientationStateFour = 0.0;
    timeStepStateFour = 3;
    validityStateFour = ValidStates{true, true, true, true,
                                     true, true, true};

    xPositionStateFive = 4.0;
    yPositionStateFive = 9.0;
    velocityStateFive = 20.0;
    accelerationStateFive = 1.0;
    lonPositionStateFive = 4.0;
    latPositionStateFive = 9.0;
    orientationStateFive = 0.0;
    timeStepStateFive = 0;
    validityStateFour = ValidStates{true, true, true, true,
                                    true, true, true};

    xPositionStateSix = 11.0;
    yPositionStateSix  = 1.0;
    velocityStateSix  = 40.0;
    accelerationStateSix  = -1.0;
    lonPositionStateSix  = 11.0;
    latPositionStateSix  = 1.0;
    orientationStateSix  = 0.0;
    timeStepStateSix  = 1;
    validityStateFour = ValidStates{true, true, true, true,
                                    true, true, true};

    stateOne = State();
    stateOne.setAcceleration(accelerationStateOne);
    stateOne.setLatPosition(latPositionStateOne);
    stateOne.setLonPosition(lonPositionStateOne);
    stateOne.setOrientation(orientationStateOne);
    stateOne.setTimeStep(timeStepStateOne);
    stateOne.setVelocity(velocityStateOne);
    stateOne.setXPosition(xPositionStateOne);
    stateOne.setYPosition(yPositionStateOne);

    stateTwo = State(timeStepStateTwo, xPositionStateTwo, yPositionStateTwo, velocityStateTwo,
                     accelerationStateTwo, orientationStateTwo);
    stateTwo.setLonPosition(lonPositionStateTwo);
    stateTwo.setLatPosition(latPositionStateTwo);

    stateThree = State(timeStepStateThree, xPositionStateThree,
                       yPositionStateThree, velocityStateThree,
                       accelerationStateThree, orientationStateThree,
                       lonPositionStateThree, latPositionStateThree);

    stateFour = State(timeStepStateFour, xPositionStateFour, yPositionStateFour,
                      velocityStateFour,accelerationStateFour, orientationStateFour,
                      lonPositionStateFour, latPositionStateFour);

    stateFive = State(timeStepStateFive, xPositionStateFive, yPositionStateFive,
                      velocityStateFive,accelerationStateFive, orientationStateFive,
                      lonPositionStateFive, latPositionStateFive);

    stateSix = State(timeStepStateSix, xPositionStateSix, yPositionStateSix,
                      velocityStateSix, accelerationStateSix, orientationStateSix,
                      lonPositionStateSix, latPositionStateSix);
}

void StateTest::SetUp() {
    setUpLane();
    setUpStates();
}

TEST_F(StateTest, InitializationComplete){
    EXPECT_EQ(stateOne.getTimeStep(), timeStepStateOne);
    EXPECT_EQ(stateOne.getXPosition(), xPositionStateOne);
    EXPECT_EQ(stateOne.getYPosition(), yPositionStateOne);
    EXPECT_EQ(stateOne.getVelocity(), velocityStateOne);
    EXPECT_EQ(stateOne.getAcceleration(), accelerationStateOne);
    EXPECT_EQ(stateOne.getLonPosition(), lonPositionStateOne);
    EXPECT_EQ(stateOne.getLatPosition(), latPositionStateOne);
    EXPECT_EQ(stateOne.getOrientation(), orientationStateOne);
    EXPECT_EQ(stateOne.getValidStates().xPosition, validityStateOne.xPosition);
    EXPECT_EQ(stateOne.getValidStates().yPosition, validityStateOne.yPosition);
    EXPECT_EQ(stateOne.getValidStates().velocity, validityStateOne.velocity);
    EXPECT_EQ(stateOne.getValidStates().acceleration, validityStateOne.acceleration);
    EXPECT_EQ(stateOne.getValidStates().lonPosition, validityStateOne.lonPosition);
    EXPECT_EQ(stateOne.getValidStates().latPosition, validityStateOne.latPosition);
    EXPECT_EQ(stateOne.getValidStates().orientation, validityStateOne.orientation);

    EXPECT_EQ(stateTwo.getTimeStep(), timeStepStateTwo);
    EXPECT_EQ(stateTwo.getXPosition(), xPositionStateTwo);
    EXPECT_EQ(stateTwo.getYPosition(), yPositionStateTwo);
    EXPECT_EQ(stateTwo.getVelocity(), velocityStateTwo);
    EXPECT_EQ(stateTwo.getAcceleration(), accelerationStateTwo);
    EXPECT_EQ(stateTwo.getLonPosition(), lonPositionStateTwo);
    EXPECT_EQ(stateTwo.getLatPosition(), latPositionStateTwo);
    EXPECT_EQ(stateTwo.getOrientation(), orientationStateTwo);
    EXPECT_EQ(stateTwo.getValidStates().xPosition, validityStateTwo.xPosition);
    EXPECT_EQ(stateTwo.getValidStates().yPosition, validityStateTwo.yPosition);
    EXPECT_EQ(stateTwo.getValidStates().velocity, validityStateTwo.velocity);
    EXPECT_EQ(stateTwo.getValidStates().acceleration, validityStateTwo.acceleration);
    EXPECT_EQ(stateTwo.getValidStates().lonPosition, validityStateTwo.lonPosition);
    EXPECT_EQ(stateTwo.getValidStates().latPosition, validityStateTwo.latPosition);
    EXPECT_EQ(stateTwo.getValidStates().orientation, validityStateTwo.orientation);

    EXPECT_EQ(stateThree.getTimeStep(), timeStepStateThree);
    EXPECT_EQ(stateThree.getXPosition(), xPositionStateThree);
    EXPECT_EQ(stateThree.getYPosition(), yPositionStateThree);
    EXPECT_EQ(stateThree.getVelocity(), velocityStateThree);
    EXPECT_EQ(stateThree.getAcceleration(), accelerationStateThree);
    EXPECT_EQ(stateThree.getLonPosition(), lonPositionStateThree);
    EXPECT_EQ(stateThree.getLatPosition(), latPositionStateThree);
    EXPECT_EQ(stateThree.getOrientation(), orientationStateThree);
    EXPECT_EQ(stateThree.getValidStates().xPosition, validityStateThree.xPosition);
    EXPECT_EQ(stateThree.getValidStates().yPosition, validityStateThree.yPosition);
    EXPECT_EQ(stateThree.getValidStates().velocity, validityStateThree.velocity);
    EXPECT_EQ(stateThree.getValidStates().acceleration, validityStateThree.acceleration);
    EXPECT_EQ(stateThree.getValidStates().lonPosition, validityStateThree.lonPosition);
    EXPECT_EQ(stateThree.getValidStates().latPosition, validityStateThree.latPosition);
    EXPECT_EQ(stateThree.getValidStates().orientation, validityStateThree.orientation);
}

TEST_F(StateTest, ConvertPointToCurvilinear){
    stateOne.setXPosition(3.0);
    stateOne.setYPosition(0.5);
    stateOne.convertPointToCurvilinear(laneOne->getCurvilinearCoordinateSystem());
    EXPECT_EQ(stateOne.getLonPosition(), 3.0);
    EXPECT_EQ(stateOne.getLatPosition(), 0.0);
    stateOne.setXPosition(2.5);
    stateOne.setYPosition(-0.75);
    stateOne.convertPointToCurvilinear(laneOne->getCurvilinearCoordinateSystem());
    EXPECT_EQ(stateOne.getLonPosition(), 2.5);
    EXPECT_EQ(stateOne.getLatPosition(), -1.25);
}