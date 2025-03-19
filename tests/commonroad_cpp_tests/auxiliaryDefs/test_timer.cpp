#include "test_timer.h"
#include "commonroad_cpp/auxiliaryDefs/timer.h"

TEST_F(TestTimer, TimerStartStop) {
    Timer evaluationTimer;
    auto startTime{Timer::start()};
    long compTime{evaluationTimer.stop(startTime)};
    EXPECT_GT(compTime, 0);
}

TEST_F(TestTimer, AddTime) {
    Timer evaluationTimer;
    evaluationTimer.addTime(10000);
    EXPECT_EQ(evaluationTimer.getTotalTime(), 10000);
}
