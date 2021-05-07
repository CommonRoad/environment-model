//
// Created by sebastian on 20.12.20.
//

#include "test_state.h"

void StateTestInitialization::setUpStates() {
    xPositionStateOne = 2.5;
    yPositionStateOne = 0.5;
    velocityStateOne = 2;
    accelerationStateOne = 1.0;
    lonPositionStateOne = 7.5;
    latPositionStateOne = 0.5;
    globalOrientationStateOne = 0.0;
    curvilinearOrientationStateOne = 0.0;
    timeStepStateOne = 0;
    validityStateOne = ValidStates{true, true, true, true, true, true, true};

    xPositionStateTwo = 3.5;
    yPositionStateTwo = 2;
    velocityStateTwo = 20.0;
    accelerationStateTwo = 0.0;
    lonPositionStateTwo = 8.5;
    latPositionStateTwo = 2.0;
    globalOrientationStateTwo = M_PI / 2;
    curvilinearOrientationStateTwo = M_PI / 2;
    timeStepStateTwo = 1;
    validityStateTwo = ValidStates{true, true, true, true, true, true, true};

    xPositionStateThree = 4.0;
    yPositionStateThree = -0.5;
    velocityStateThree = 0.0;
    accelerationStateThree = -0.6;
    lonPositionStateThree = 9.0;
    latPositionStateThree = -0.5;
    globalOrientationStateThree = 0.01;
    curvilinearOrientationStateThree = 0.01;
    timeStepStateThree = 2;
    validityStateThree = ValidStates{true, true, true, true, true, true, true};

    xPositionStateFour = 5.0;
    yPositionStateFour = -0.0;
    velocityStateFour = 50.0;
    accelerationStateFour = 0.0;
    lonPositionStateFour = 10;
    latPositionStateFour = 0;
    globalOrientationStateFour = -0.01;
    curvilinearOrientationStateFour = -0.01;
    timeStepStateFour = 3;
    validityStateFour = ValidStates{true, true, true, true, true, true, true};

    xPositionStateFive = 4.0;
    yPositionStateFive = 9.0;
    velocityStateFive = 20.0;
    accelerationStateFive = 1.0;
    lonPositionStateFive = 9.0;
    latPositionStateFive = 9.0;
    globalOrientationStateFive = 0.00;
    curvilinearOrientationStateFive = 0.0;
    timeStepStateFive = 4;
    validityStateFour = ValidStates{true, true, true, true, true, true, true};

    xPositionStateSix = 11.0;
    yPositionStateSix = 1.0;
    velocityStateSix = 40.0;
    accelerationStateSix = -1.0;
    lonPositionStateSix = 16.0;
    latPositionStateSix = 1.0;
    globalOrientationStateSix = 0.0;
    curvilinearOrientationStateSix = 0.0;
    timeStepStateSix = 5;
    validityStateFour = ValidStates{true, true, true, true, true, true, true};

    stateOne = std::make_shared<State>();
    stateOne->setAcceleration(accelerationStateOne);
    stateOne->setLatPosition(latPositionStateOne);
    stateOne->setLonPosition(lonPositionStateOne);
    stateOne->setGlobalOrientation(globalOrientationStateOne);
    stateOne->setCurvilinearOrientation(curvilinearOrientationStateOne);
    stateOne->setTimeStep(timeStepStateOne);
    stateOne->setVelocity(velocityStateOne);
    stateOne->setXPosition(xPositionStateOne);
    stateOne->setYPosition(yPositionStateOne);
    stateOne->setCurvilinearOrientation(curvilinearOrientationStateOne);

    stateTwo = std::make_shared<State>(timeStepStateTwo, xPositionStateTwo, yPositionStateTwo, velocityStateTwo,
                                       accelerationStateTwo, globalOrientationStateTwo);
    stateTwo->setLonPosition(lonPositionStateTwo);
    stateTwo->setLatPosition(latPositionStateTwo);
    stateTwo->setCurvilinearOrientation(curvilinearOrientationStateTwo);

    stateThree = std::make_shared<State>(
        timeStepStateThree, xPositionStateThree, yPositionStateThree, velocityStateThree, accelerationStateThree,
        globalOrientationStateThree, curvilinearOrientationStateThree, lonPositionStateThree, latPositionStateThree);

    stateFour = std::make_shared<State>(timeStepStateFour, xPositionStateFour, yPositionStateFour, velocityStateFour,
                                        accelerationStateFour, globalOrientationStateFour,
                                        curvilinearOrientationStateFour, lonPositionStateFour, latPositionStateFour);

    stateFive = std::make_shared<State>(timeStepStateFive, xPositionStateFive, yPositionStateFive, velocityStateFive,
                                        accelerationStateFive, globalOrientationStateFive,
                                        curvilinearOrientationStateFive, lonPositionStateFive, latPositionStateFive);

    stateSix = std::make_shared<State>(timeStepStateSix, xPositionStateSix, yPositionStateSix, velocityStateSix,
                                       accelerationStateSix, globalOrientationStateSix, curvilinearOrientationStateSix,
                                       lonPositionStateSix, latPositionStateSix);
}

void StateTest::SetUp() {
    setUpLane();
    setUpStates();
}

TEST_F(StateTest, InitializationComplete) {
    EXPECT_EQ(stateOne->getTimeStep(), timeStepStateOne);
    EXPECT_EQ(stateOne->getXPosition(), xPositionStateOne);
    EXPECT_EQ(stateOne->getYPosition(), yPositionStateOne);
    EXPECT_EQ(stateOne->getVelocity(), velocityStateOne);
    EXPECT_EQ(stateOne->getAcceleration(), accelerationStateOne);
    EXPECT_EQ(stateOne->getLonPosition(), lonPositionStateOne);
    EXPECT_EQ(stateOne->getLatPosition(), latPositionStateOne);
    EXPECT_EQ(stateOne->getGlobalOrientation(), globalOrientationStateOne);
    EXPECT_EQ(stateOne->getValidStates().xPosition, validityStateOne.xPosition);
    EXPECT_EQ(stateOne->getValidStates().yPosition, validityStateOne.yPosition);
    EXPECT_EQ(stateOne->getValidStates().velocity, validityStateOne.velocity);
    EXPECT_EQ(stateOne->getValidStates().acceleration, validityStateOne.acceleration);
    EXPECT_EQ(stateOne->getValidStates().lonPosition, validityStateOne.lonPosition);
    EXPECT_EQ(stateOne->getValidStates().latPosition, validityStateOne.latPosition);
    EXPECT_EQ(stateOne->getValidStates().globalOrientation, validityStateOne.globalOrientation);

    EXPECT_EQ(stateTwo->getTimeStep(), timeStepStateTwo);
    EXPECT_EQ(stateTwo->getXPosition(), xPositionStateTwo);
    EXPECT_EQ(stateTwo->getYPosition(), yPositionStateTwo);
    EXPECT_EQ(stateTwo->getVelocity(), velocityStateTwo);
    EXPECT_EQ(stateTwo->getAcceleration(), accelerationStateTwo);
    EXPECT_EQ(stateTwo->getLonPosition(), lonPositionStateTwo);
    EXPECT_EQ(stateTwo->getLatPosition(), latPositionStateTwo);
    EXPECT_EQ(stateTwo->getGlobalOrientation(), globalOrientationStateTwo);
    EXPECT_EQ(stateTwo->getValidStates().xPosition, validityStateTwo.xPosition);
    EXPECT_EQ(stateTwo->getValidStates().yPosition, validityStateTwo.yPosition);
    EXPECT_EQ(stateTwo->getValidStates().velocity, validityStateTwo.velocity);
    EXPECT_EQ(stateTwo->getValidStates().acceleration, validityStateTwo.acceleration);
    EXPECT_EQ(stateTwo->getValidStates().lonPosition, validityStateTwo.lonPosition);
    EXPECT_EQ(stateTwo->getValidStates().latPosition, validityStateTwo.latPosition);
    EXPECT_EQ(stateTwo->getValidStates().globalOrientation, validityStateTwo.globalOrientation);

    EXPECT_EQ(stateThree->getTimeStep(), timeStepStateThree);
    EXPECT_EQ(stateThree->getXPosition(), xPositionStateThree);
    EXPECT_EQ(stateThree->getYPosition(), yPositionStateThree);
    EXPECT_EQ(stateThree->getVelocity(), velocityStateThree);
    EXPECT_EQ(stateThree->getAcceleration(), accelerationStateThree);
    EXPECT_EQ(stateThree->getLonPosition(), lonPositionStateThree);
    EXPECT_EQ(stateThree->getLatPosition(), latPositionStateThree);
    EXPECT_EQ(stateThree->getGlobalOrientation(), globalOrientationStateThree);
    EXPECT_EQ(stateThree->getValidStates().xPosition, validityStateThree.xPosition);
    EXPECT_EQ(stateThree->getValidStates().yPosition, validityStateThree.yPosition);
    EXPECT_EQ(stateThree->getValidStates().velocity, validityStateThree.velocity);
    EXPECT_EQ(stateThree->getValidStates().acceleration, validityStateThree.acceleration);
    EXPECT_EQ(stateThree->getValidStates().lonPosition, validityStateThree.lonPosition);
    EXPECT_EQ(stateThree->getValidStates().latPosition, validityStateThree.latPosition);
    EXPECT_EQ(stateThree->getValidStates().globalOrientation, validityStateThree.globalOrientation);
}
