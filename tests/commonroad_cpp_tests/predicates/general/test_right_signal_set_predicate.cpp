#include "test_right_signal_set_predicate.h"
#include "../utils_predicate_test.h"

void RightSignalSetPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 10, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 0, 10, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 2, 10, 0, 0, 0, 20, 2);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
                                                         std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                                         std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {}, 0.1));
}

TEST_F(RightSignalSetPredicateTest, AlwaysOff) {
    std::shared_ptr<SignalState> signalStateZero =
        std::make_shared<SignalState>(0, false, false, false, false, false, false);
    std::shared_ptr<SignalState> signalStateOne =
        std::make_shared<SignalState>(1, false, false, false, false, false, false);
    std::shared_ptr<SignalState> signalStateTwo =
        std::make_shared<SignalState>(2, false, false, false, false, false, false);
    egoVehicle->appendSignalStateToSeries(signalStateZero);
    egoVehicle->appendSignalStateToSeries(signalStateOne);
    egoVehicle->appendSignalStateToSeries(signalStateTwo);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle));
}

TEST_F(RightSignalSetPredicateTest, AlwaysOn) {
    std::shared_ptr<SignalState> signalStateZero =
        std::make_shared<SignalState>(0, false, false, true, false, false, false);
    std::shared_ptr<SignalState> signalStateOne =
        std::make_shared<SignalState>(1, false, false, true, false, false, false);
    std::shared_ptr<SignalState> signalStateTwo =
        std::make_shared<SignalState>(2, false, false, true, false, false, false);
    egoVehicle->appendSignalStateToSeries(signalStateZero);
    egoVehicle->appendSignalStateToSeries(signalStateOne);
    egoVehicle->appendSignalStateToSeries(signalStateTwo);
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle));
}

TEST_F(RightSignalSetPredicateTest, FirstOffThanOn) {
    std::shared_ptr<SignalState> signalStateZero =
        std::make_shared<SignalState>(0, false, false, false, false, false, false);
    std::shared_ptr<SignalState> signalStateOne =
        std::make_shared<SignalState>(1, false, false, false, false, false, false);
    std::shared_ptr<SignalState> signalStateTwo =
        std::make_shared<SignalState>(2, false, false, true, false, false, false);
    egoVehicle->appendSignalStateToSeries(signalStateZero);
    egoVehicle->appendSignalStateToSeries(signalStateOne);
    egoVehicle->appendSignalStateToSeries(signalStateTwo);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle));
}

TEST_F(RightSignalSetPredicateTest, FirstOnThanOff) {
    std::shared_ptr<SignalState> signalStateZero =
        std::make_shared<SignalState>(2, false, false, false, false, false, false);
    std::shared_ptr<SignalState> signalStateOne =
        std::make_shared<SignalState>(1, false, false, false, false, false, false);
    std::shared_ptr<SignalState> signalStateTwo =
        std::make_shared<SignalState>(0, false, false, true, false, false, false);
    egoVehicle->appendSignalStateToSeries(signalStateZero);
    egoVehicle->appendSignalStateToSeries(signalStateOne);
    egoVehicle->appendSignalStateToSeries(signalStateTwo);
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle));
}

TEST_F(RightSignalSetPredicateTest, Alternating) {
    std::shared_ptr<SignalState> signalStateZero =
        std::make_shared<SignalState>(2, false, false, true, false, false, false);
    std::shared_ptr<SignalState> signalStateOne =
        std::make_shared<SignalState>(1, false, false, false, false, false, false);
    std::shared_ptr<SignalState> signalStateTwo =
        std::make_shared<SignalState>(0, false, false, true, false, false, false);
    egoVehicle->appendSignalStateToSeries(signalStateZero);
    egoVehicle->appendSignalStateToSeries(signalStateOne);
    egoVehicle->appendSignalStateToSeries(signalStateTwo);
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle));
}

TEST_F(RightSignalSetPredicateTest, SignalHistoryNotSet) {
    EXPECT_THROW(pred.booleanEvaluation(0, world, egoVehicle), std::logic_error);
    EXPECT_THROW(pred.booleanEvaluation(1, world, egoVehicle), std::logic_error);
    EXPECT_THROW(pred.booleanEvaluation(2, world, egoVehicle), std::logic_error);
}

TEST_F(RightSignalSetPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(RightSignalSetPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}