#include "test_signal_state.h"

void SignalStateTestInitialization::setUpSignalStates() {
    timeStepSignalStateOne = 0;
    hornSignalStateOne = true;
    indicatorLeftSignalStateOne = false;
    indicatorRightSignalStateOne = true;
    brakingLightsSignalStateOne = false;
    hazardWarningLightsSignalStateOne = true;
    flashingBlueLightsSignalStateOne = false;

    timeStepSignalStateTwo = 1;
    hornSignalStateTwo = false;
    indicatorLeftSignalStateTwo = true;
    indicatorRightSignalStateTwo = false;
    brakingLightsSignalStateTwo = true;
    hazardWarningLightsSignalStateTwo = false;
    flashingBlueLightsSignalStateTwo = true;

    signalStateOne = std::make_shared<SignalState>(SignalState(
        timeStepSignalStateOne, hornSignalStateOne, indicatorLeftSignalStateOne, indicatorRightSignalStateOne,
        brakingLightsSignalStateOne, hazardWarningLightsSignalStateOne, flashingBlueLightsSignalStateOne));
    signalStateTwo = std::make_shared<SignalState>(SignalState(
        timeStepSignalStateTwo, hornSignalStateTwo, indicatorLeftSignalStateTwo, indicatorRightSignalStateTwo,
        brakingLightsSignalStateTwo, hazardWarningLightsSignalStateTwo, flashingBlueLightsSignalStateTwo));
}

void SignalStateTest::SetUp() { setUpSignalStates(); }

TEST_F(SignalStateTest, TestInitializationComplete) {

    EXPECT_EQ(signalStateOne->getTimeStep(), timeStepSignalStateOne);
    EXPECT_EQ(signalStateOne->isHorn(), hornSignalStateOne);
    EXPECT_EQ(signalStateOne->isIndicatorLeft(), indicatorLeftSignalStateOne);
    EXPECT_EQ(signalStateOne->isIndicatorRight(), indicatorRightSignalStateOne);
    EXPECT_EQ(signalStateOne->isBrakingLights(), brakingLightsSignalStateOne);
    EXPECT_EQ(signalStateOne->isHazardWarningLights(), hazardWarningLightsSignalStateOne);
    EXPECT_EQ(signalStateOne->isFlashingBlueLights(), flashingBlueLightsSignalStateOne);

    EXPECT_EQ(signalStateTwo->getTimeStep(), timeStepSignalStateTwo);
    EXPECT_EQ(signalStateTwo->isHorn(), hornSignalStateTwo);
    EXPECT_EQ(signalStateTwo->isIndicatorLeft(), indicatorLeftSignalStateTwo);
    EXPECT_EQ(signalStateTwo->isIndicatorRight(), indicatorRightSignalStateTwo);
    EXPECT_EQ(signalStateTwo->isBrakingLights(), brakingLightsSignalStateTwo);
    EXPECT_EQ(signalStateTwo->isHazardWarningLights(), hazardWarningLightsSignalStateTwo);
    EXPECT_EQ(signalStateTwo->isFlashingBlueLights(), flashingBlueLightsSignalStateTwo);
}

TEST_F(SignalStateTest, TestSetter) {
    signalStateOne->setTimeStep(timeStepSignalStateOne);
    signalStateOne->setHorn(hornSignalStateOne);
    signalStateOne->setIndicatorLeft(indicatorLeftSignalStateOne);
    signalStateOne->setIndicatorRight(indicatorRightSignalStateOne);
    signalStateOne->setBrakingLights(brakingLightsSignalStateOne);
    signalStateOne->setHazardWarningLights(hazardWarningLightsSignalStateOne);
    signalStateOne->setFlashingBlueLights(flashingBlueLightsSignalStateOne);
    signalStateOne->setBrakingLights(brakingLightsSignalStateOne);
    EXPECT_EQ(signalStateOne->getTimeStep(), timeStepSignalStateOne);
    EXPECT_EQ(signalStateOne->isHorn(), hornSignalStateOne);
    EXPECT_EQ(signalStateOne->isIndicatorLeft(), indicatorLeftSignalStateOne);
    EXPECT_EQ(signalStateOne->isIndicatorRight(), indicatorRightSignalStateOne);
    EXPECT_EQ(signalStateOne->isBrakingLights(), brakingLightsSignalStateOne);
    EXPECT_EQ(signalStateOne->isHazardWarningLights(), hazardWarningLightsSignalStateOne);
    EXPECT_EQ(signalStateOne->isFlashingBlueLights(), flashingBlueLightsSignalStateOne);

    signalStateTwo->setTimeStep(timeStepSignalStateTwo);
    signalStateTwo->setHorn(hornSignalStateTwo);
    signalStateTwo->setIndicatorLeft(indicatorLeftSignalStateTwo);
    signalStateTwo->setIndicatorRight(indicatorRightSignalStateTwo);
    signalStateTwo->setBrakingLights(brakingLightsSignalStateTwo);
    signalStateTwo->setHazardWarningLights(hazardWarningLightsSignalStateTwo);
    signalStateTwo->setFlashingBlueLights(flashingBlueLightsSignalStateTwo);
    signalStateTwo->setBrakingLights(brakingLightsSignalStateTwo);
    EXPECT_EQ(signalStateTwo->getTimeStep(), timeStepSignalStateTwo);
    EXPECT_EQ(signalStateTwo->isHorn(), hornSignalStateTwo);
    EXPECT_EQ(signalStateTwo->isIndicatorLeft(), indicatorLeftSignalStateTwo);
    EXPECT_EQ(signalStateTwo->isIndicatorRight(), indicatorRightSignalStateTwo);
    EXPECT_EQ(signalStateTwo->isBrakingLights(), brakingLightsSignalStateTwo);
    EXPECT_EQ(signalStateTwo->isHazardWarningLights(), hazardWarningLightsSignalStateTwo);
    EXPECT_EQ(signalStateTwo->isFlashingBlueLights(), flashingBlueLightsSignalStateTwo);
}
