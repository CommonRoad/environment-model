#include "test_brakes_stronger_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/obstacle.h"

void BrakesStrongerPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 10, -2, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 10, 2, 10, -1, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 0, 2, 10, -1, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 10, 2, 10, -2, 0);

    std::shared_ptr<State> stateTwoObstacleOne{std::make_shared<State>()};
    stateTwoObstacleOne->setTimeStep(2);
    stateTwoObstacleOne->setXPosition(20);
    stateTwoObstacleOne->setYPosition(2);
    stateTwoObstacleOne->setVelocity(10.1);
    stateTwoObstacleOne->setGlobalOrientation(0);
    std::shared_ptr<State> stateTwoObstacleTwo{std::make_shared<State>()};
    stateTwoObstacleTwo->setTimeStep(2);
    stateTwoObstacleTwo->setXPosition(20);
    stateTwoObstacleTwo->setYPosition(2);
    stateTwoObstacleTwo->setVelocity(10.1);
    stateTwoObstacleTwo->setGlobalOrientation(0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 30, 2, 10, 2, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 40, 2, 10, 1, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));
    auto roadNetwork{utils_predicate_test::create_road_network()};

    world =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne},
                                      std::vector<std::shared_ptr<Obstacle>>{obstacleTwo}, 0.1));
}

TEST_F(BrakesStrongerPredicateTest, BooleanEvaluation) {
    std::vector<std::string> opt{"0.0"};
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, obstacleTwo, opt));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne, obstacleTwo, opt));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, obstacleTwo, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne, obstacleTwo, opt));
}

TEST_F(BrakesStrongerPredicateTest, ConstraintEvaluation) {
    std::vector<std::string> opt{"0.0"};
    EXPECT_EQ(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo, opt).realValuedConstraint, -1);
    EXPECT_EQ(pred.constraintEvaluation(1, world, obstacleOne, obstacleTwo, opt).realValuedConstraint, -2);
    EXPECT_EQ(pred.constraintEvaluation(2, world, obstacleOne, obstacleTwo, opt).realValuedConstraint, 0);
    EXPECT_EQ(pred.constraintEvaluation(3, world, obstacleOne, obstacleTwo, opt).realValuedConstraint, 0);
}

TEST_F(BrakesStrongerPredicateTest, RobustEvaluation) {
    std::vector<std::string> opt{"0.0"};
    EXPECT_EQ(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo, opt), 1);
    EXPECT_EQ(pred.robustEvaluation(1, world, obstacleOne, obstacleTwo, opt), -1);
    EXPECT_NEAR(pred.robustEvaluation(2, world, obstacleOne, obstacleTwo, opt), -1, 0.0001);
    EXPECT_EQ(pred.robustEvaluation(3, world, obstacleOne, obstacleTwo, opt), -2);
}
